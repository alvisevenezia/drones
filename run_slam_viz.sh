#!/usr/bin/env bash
# =====================================================================
#  EXECUTABLE UNIQUE — Double SLAM drone + visualisation RViz
#  - Gazebo en HEADLESS (pas de fenetre -> economise le GPU)
#  - VIO (rgbd_odometry) = odometrie commune camera RGB (+IMU)
#  - Carte A : ToF dense 2304 (48x48) -> /slam_map_tof    (orange, repere map corrige)
#  - Carte B : camera RGB-D (dense)   -> /rtabmap/cloud_map
#  - RViz montre : camera du drone + position (TF) + les 2 cartes
#
#  Usage : ./run_slam_viz.sh
#  La console pxh> s'affiche dans CE terminal :
#     commander arm -f      puis    commander takeoff
#  Vole dans l'entrepot -> les 2 cartes se construisent dans RViz.
#  Ctrl+C (ou 'shutdown' dans pxh>) pour tout arreter.
# =====================================================================
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

D="${D:-$HOME/Documents/drones}"
PX4_HOME="${PX4_HOME:-$D/PX4-Autopilot}"

source "$D/install/setup.bash"
PIDS=()
cleanup() {
  echo ""
  echo "==> Arret de tous les composants..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -9 -f '[g]z sim' 2>/dev/null
  pkill -9 -x px4 2>/dev/null
  pkill -9 -f '[p]arameter_bridge' 2>/dev/null
  pkill -9 -f '[M]icroXRCEAgent' 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "==> [1] Agent uXRCE-DDS (PX4 <-> ROS)"
MicroXRCEAgent udp4 -p 8888 >/tmp/agent.log 2>&1 & PIDS+=($!)

echo "==> [2] Pont capteurs gz -> ROS (RGB, depth, IMU, ToF)"
bash "$D/run_camera_bridge.sh" >/tmp/cambridge.log 2>&1 & PIDS+=($!)

echo "==> [2b] Monitoring runtime (RTF, CPU, RAM, GPU)"
python3 "$D/monitor_runtime.py" >/tmp/runtime_monitor.log 2>&1 & PIDS+=($!)

echo "==> [3] TF statique base_link -> camera_link (optique)"
ros2 run tf2_ros static_transform_publisher \
  --x 0.12 --y 0 --z 0.24 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
  --frame-id base_link --child-frame-id camera_link >/tmp/statictf.log 2>&1 & PIDS+=($!)

echo "==> [3b] TF statique camera_link -> tof_link (VL53L9CX co-localise avec la camera, repere FLU x-avant)"
ros2 run tf2_ros static_transform_publisher \
  --qx 0.5 --qy -0.5 --qz 0.5 --qw 0.5 \
  --frame-id camera_link --child-frame-id tof_link >/tmp/statictf_tof.log 2>&1 & PIDS+=($!)

echo "==> [3c] Profondeur ToF -> image depth (pour la loop-closure RGB-D de rtabmap)"
ros2 run rtabmap_util pointcloud_to_depthimage --ros-args \
  -p use_sim_time:=true \
  -p fixed_frame_id:=base_link \
  -p decimation:=1 \
  -p fill_holes_size:=8 \
  -p fill_holes_error:=0.5 \
  -p upscale:=true \
  -p approx:=true \
  -p wait_for_transform:=0.2 \
  -r cloud:=/tof/points \
  -r camera_info:=/camera/rgb/camera_info \
  -r image:=/camera/depth/image_raw >/tmp/tof_depth.log 2>&1 & PIDS+=($!)

echo "==> [3c-agg] Agregation des 5 VL53L9CX (avant/arriere/gauche/droite/dessous) -> /tof/points (base_link)"
python3 "$D/tof_aggregate.py" --ros-args -p use_sim_time:=true >/tmp/tof_agg.log 2>&1 & PIDS+=($!)

echo "==> [3d] Filtre NaN du nuage ToF (depth_camera leogue -> dense) -> /tof/points_dense"
python3 "$D/cloud_denan.py" --ros-args -p use_sim_time:=true >/tmp/denan.log 2>&1 & PIDS+=($!)

echo "==> [4] Odometrie ICP(ToF L9CX)-inertielle : icp_odometry (nuage + IMU) -> /odom + TF odom->base_link"
ros2 run rtabmap_odom icp_odometry --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p publish_tf:=true \
  -p wait_imu_to_init:=false \
  -p guess_frame_id:=odom \
  -p Icp/VoxelSize:="'0.05'" \
  -p Icp/PointToPlane:="'true'" \
  -p Icp/PointToPlaneK:="'20'" \
  -p Icp/Iterations:="'10'" \
  -p Icp/MaxCorrespondenceDistance:="'1.0'" \
  -p Icp/Epsilon:="'0.001'" \
  -p Icp/MaxTranslation:="'2.0'" \
  -p Odom/ScanKeyFrameThr:="'0.6'" \
  -p Odom/GuessMotion:="'true'" \
  -p Odom/ResetCountdown:="'1'" \
  -p OdomF2M/ScanMaxSize:="'15000'" \
  -r scan:=/scan_dummy_unused \
  -r scan_cloud:=/tof/points_dense \
  -r imu:=/camera/imu >/tmp/vio.log 2>&1 & PIDS+=($!)

# --- EKF (robot_localization) DESACTIVE : degradait l'odometrie (drift ~25% + rtabmap wm=0).
#     L'ICP brut ci-dessus (publish_tf:=true, /odom direct) est bien meilleur (~1-2%).
#     Pour re-tester l'EKF : icp -> publish_tf:=false + "-r odom:=/icp_odom", puis decommenter :
# ros2 run robot_localization ekf_node --ros-args --params-file "$D/ekf.yaml" \
#   -p use_sim_time:=true -r /odometry/filtered:=/odom >/tmp/ekf.log 2>&1 & PIDS+=($!)
# --- Repli FAST-LIO (compile, mais PCL ingere mal le ToF epars ; garde en option) : ---
# ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
#   --frame-id odom --child-frame-id camera_init >/tmp/tf_ci.log 2>&1 & PIDS+=($!)
# ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
#   --frame-id body --child-frame-id base_link >/tmp/tf_body.log 2>&1 & PIDS+=($!)
# ros2 run fast_lio fastlio_mapping --ros-args --params-file "$D/fastlio_vl53l9cx.yaml" \
#   -p use_sim_time:=true -r /Odometry:=/odom >/tmp/vio.log 2>&1 & PIDS+=($!)
# --- Repli fiable (si la VIO decroche) : commente le bloc rgbd_odometry ci-dessus, decommente : ---
# python3 "$D/px4_odom.py" --ros-args -p use_sim_time:=true >/tmp/odom.log 2>&1 & PIDS+=($!)

echo "==> [5] Carte B : RTAB-Map (nuage ToF + RGB pour loop-closure)"
ros2 run rtabmap_slam rtabmap -d --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=base_link \
  -p subscribe_depth:=true \
  -p subscribe_rgb:=true \
  -p subscribe_scan_cloud:=true \
  -p approx_sync:=true \
  -p Reg/Strategy:="'1'" \
  -p Vis/MinInliers:="'4'" \
  -p RGBD/ProximityBySpace:="'true'" \
  -p RGBD/ProximityPathMaxNeighbors:="'10'" \
  -p Mem/BinDataKept:="'true'" \
  -p Mem/NotLinkedNodesKept:="'false'" \
  -p RGBD/ProximityMaxGraphDepth:="'0'" \
  -p RGBD/OptimizeFromGraphEnd:="'false'" \
  -p RGBD/OptimizeMaxError:="'0'" \
  -p Mem/STMSize:="'30'" \
  -p Icp/VoxelSize:="'0.05'" \
  -p Icp/PointToPlane:="'false'" \
  -r rgb/image:=/camera/rgb/image_raw \
  -r depth/image:=/camera/depth/image_raw \
  -r rgb/camera_info:=/camera/rgb/camera_info \
  -r scan_cloud:=/tof/points_dense \
  -r odom:=/odom >/tmp/rtabmap.log 2>&1 & PIDS+=($!)

echo "==> [6] Carte A : mapper ToF 2304 (repere map corrige par RTAB-Map)"
python3 "$D/slam_mapper.py" --ros-args -p use_sim_time:=true >/tmp/slam_vl53.log 2>&1 &

echo "==> [7] Dashboard Qt : telemetry gauche + 3D reconstruction centre + carte/goals droite"
ros2 run dual_rviz_jazzy dual_rviz --ros-args -p use_sim_time:=true >/tmp/slam_dashboard.log 2>&1 &
sleep 3
echo ""
echo "==> [7b] Eval drift : VIO /odom vs verite PX4 -> /vio_drift + /truth_marker (ROUGE)"
python3 "$D/drift_eval.py" --ros-args -p use_sim_time:=true >/tmp/drift.log 2>&1 & PIDS+=($!)

echo "==> [7c] Exploration frontieres : grille 2D + but -> /explore/grid + /explore/goal"
python3 "$D/frontier_explore.py" --ros-args -p use_sim_time:=true >/tmp/frontier.log 2>&1 & PIDS+=($!)

#echo "==> [7d] Controleur d'exploration : patrol.py suit /explore/goal (OFFBOARD). NE PAS lancer le teleop en meme temps !"
#python3 "$D/patrol.py" --ros-args -p use_sim_time:=true >/tmp/patrol.log 2>&1 & PIDS+=($!)

echo "==> [8] PX4 SITL + Gazebo (headless) — console pxh> ci-dessous"
echo "        Tape :   commander arm -f    puis    commander takeoff"
echo ""
cd "$PX4_HOME" && PX4_GZ_WORLD=tugbot_depot HEADLESS=1 make px4_sitl gz_x500_depth