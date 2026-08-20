#!/usr/bin/env bash
# =====================================================================
# RUN SLAM — PX4 + Gazebo + ROS 2 + double SLAM + RViz
#
# Ordre :
#   1. ROS 2
#   2. MicroXRCE-DDS Agent
#   3. PX4 + Gazebo
#   4. Attente des topics Gazebo / ROS
#   5. Bridges capteurs
#   6. TF
#   7. Traitement ToF
#   8. ICP odometry
#   9. RTAB-Map
#  10. SLAM ToF
#  11. RViz / dashboard
#  12. Exploration / évaluation
#
# Gazebo est visible par défaut.
# Pour du headless :
#   HEADLESS=1 ./run_slam_viz.sh
#
# =====================================================================

set -Eeo pipefail

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

D="${D:-$HOME/Documents/drones}"
PX4_HOME="${PX4_HOME:-$D/PX4-Autopilot}"

WORLD="${PX4_GZ_WORLD:-tugbot_depot}"
MODEL="${PX4_SIM_MODEL:-gz_x500_depth}"

HEADLESS="${HEADLESS:-0}"

PIDS=()

# ---------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

if [ -f "$D/install/setup.bash" ]; then
    source "$D/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"

# ---------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------

cleanup() {
    local code=$?

    echo ""
    echo "============================================================"
    echo " Arrêt de la simulation"
    echo "============================================================"

    # Arrêt propre des processus que NOUS avons lancés
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    # Petite attente pour laisser mourir les processus
    sleep 1

    # Nettoyage de sécurité
    pkill -9 -f '[M]icroXRCEAgent' 2>/dev/null || true
    pkill -9 -f '[p]arameter_bridge' 2>/dev/null || true
    pkill -9 -f '[g]z sim' 2>/dev/null || true
    pkill -9 -x px4 2>/dev/null || true

    echo "Simulation arrêtée."
    exit "$code"
}

trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------
# Fonctions utilitaires
# ---------------------------------------------------------------------

wait_for_topic() {
    local topic="$1"
    local timeout="${2:-30}"

    echo "    Attente de $topic ..."

    local start
    start=$(date +%s)

    while true; do

        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            echo "    OK : $topic"
            return 0
        fi

        if (( $(date +%s) - start >= timeout )); then
            echo "    WARNING : timeout sur $topic"
            return 1
        fi

        sleep 1
    done
}

wait_for_process() {
    local pid="$1"
    local name="$2"
    local timeout="${3:-30}"

    local start
    start=$(date +%s)

    while kill -0 "$pid" 2>/dev/null; do

        if (( $(date +%s) - start >= timeout )); then
            echo "    WARNING : $name tourne toujours après ${timeout}s"
            return 0
        fi

        sleep 1
    done

    echo "    ERROR : $name s'est arrêté."
    return 1
}

# =====================================================================
# 0 — Vérifications
# =====================================================================

echo ""
echo "============================================================"
echo " Vérification environnement"
echo "============================================================"

echo "PX4_HOME : $PX4_HOME"
echo "WORLD    : $WORLD"
echo "MODEL    : $MODEL"
echo "HEADLESS : $HEADLESS"

if [ ! -d "$PX4_HOME" ]; then
    echo "ERROR : PX4_HOME introuvable : $PX4_HOME"
    exit 1
fi

if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR : Gazebo (gz) introuvable."
    exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    echo "ERROR : MicroXRCEAgent introuvable."
    exit 1
fi

# Vérification du monde
WORLD_FILE="$PX4_HOME/Tools/simulation/gz/worlds/${WORLD}.sdf"

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR : monde Gazebo introuvable :"
    echo "        $WORLD_FILE"
    exit 1
fi

# =====================================================================
# 1 — Micro XRCE DDS Agent
# =====================================================================

echo ""
echo "============================================================"
echo " [1] MicroXRCE-DDS Agent"
echo "============================================================"

MicroXRCEAgent udp4 -p 8888 \
    >/tmp/agent.log 2>&1 &

PIDS+=($!)

sleep 2

if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "ERROR : MicroXRCEAgent n'a pas démarré."
    cat /tmp/agent.log
    exit 1
fi

echo "    Agent actif sur UDP :8888"

# =====================================================================
# 2 — PX4 + Gazebo
# =====================================================================

echo ""
echo "============================================================"
echo " [2] PX4 SITL + Gazebo"
echo "============================================================"

cd "$PX4_HOME"

echo "    Model : $MODEL"
echo "    World : $WORLD"

# ---------------------------------------------------------------------
# Important :
#
# On utilise directement la cible déjà compilée.
#
# Si HEADLESS=1 :
#   Gazebo serveur uniquement.
#
# Sinon :
#   Gazebo + GUI.
# ---------------------------------------------------------------------

if [ "$HEADLESS" = "1" ]; then

    echo "    Mode : HEADLESS"

    PX4_GZ_WORLD="$WORLD" \
    PX4_SIM_MODEL="$MODEL" \
    HEADLESS=1 \
    make px4_sitl gz_x500_depth \
        >/tmp/px4.log 2>&1 &

else

    echo "    Mode : GUI"

    PX4_GZ_WORLD="$WORLD" \
    PX4_SIM_MODEL="$MODEL" \
    make px4_sitl gz_x500_depth \
        >/tmp/px4.log 2>&1 &

fi

PX4_PID=$!
PIDS+=("$PX4_PID")

echo "    PX4 PID : $PX4_PID"

# ---------------------------------------------------------------------
# On affiche les logs PX4 dans le terminal.
# ---------------------------------------------------------------------

(
    tail -f /tmp/px4.log
) &

LOG_PID=$!
PIDS+=("$LOG_PID")

# =====================================================================
# 3 — Attente PX4 / Gazebo
# =====================================================================

echo ""
echo "============================================================"
echo " [3] Attente PX4 + Gazebo"
echo "============================================================"

echo "    Attente du démarrage de Gazebo..."

sleep 5

# Vérifie que Gazebo existe
for i in {1..30}; do

    if pgrep -f '[g]z sim' >/dev/null 2>&1; then
        echo "    Gazebo détecté."
        break
    fi

    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "ERROR : PX4 s'est arrêté."
        echo ""
        cat /tmp/px4.log
        exit 1
    fi

    sleep 1

done

if ! pgrep -f '[g]z sim' >/dev/null 2>&1; then
    echo "ERROR : Gazebo n'a pas démarré."
    exit 1
fi

# =====================================================================
# 4 — Attente ROS / PX4
# =====================================================================

echo ""
echo "============================================================"
echo " [4] Attente des topics ROS"
echo "============================================================"

# /clock doit normalement arriver dès que Gazebo tourne.
wait_for_topic "/clock" 30 || true

# On attend un peu que uXRCE-DDS initialise le client PX4.
sleep 3

echo ""
echo "    Topics disponibles :"
ros2 topic list | sort

# =====================================================================
# 5 — Bridge capteurs
# =====================================================================

echo ""
echo "============================================================"
echo " [5] Bridge capteurs Gazebo -> ROS"
echo "============================================================"

bash "$D/run_camera_bridge.sh" \
    >/tmp/cambridge.log 2>&1 &

PIDS+=($!)

sleep 3

echo "    Camera bridge démarré."

echo ""
echo "    Topics capteurs :"

ros2 topic list | grep -E \
    '/camera|/tof|/vl53|/imu' \
    || true

# =====================================================================
# 6 — Monitoring
# =====================================================================

echo ""
echo "============================================================"
echo " [6] Monitoring runtime"
echo "============================================================"

python3 "$D/monitor_runtime.py" \
    >/tmp/runtime_monitor.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 7 — TF statiques
# =====================================================================

echo ""
echo "============================================================"
echo " [7] TF statiques"
echo "============================================================"

echo "==> base_link -> camera_link"

ros2 run tf2_ros static_transform_publisher \
    --x 0.12 \
    --y 0 \
    --z 0.24 \
    --qx -0.5 \
    --qy 0.5 \
    --qz -0.5 \
    --qw 0.5 \
    --frame-id base_link \
    --child-frame-id camera_link \
    >/tmp/statictf.log 2>&1 &

PIDS+=($!)

echo "==> camera_link -> tof_link"

ros2 run tf2_ros static_transform_publisher \
    --qx 0.5 \
    --qy -0.5 \
    --qz 0.5 \
    --qw 0.5 \
    --frame-id camera_link \
    --child-frame-id tof_link \
    >/tmp/statictf_tof.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 8 — Agrégation ToF
# =====================================================================

echo ""
echo "============================================================"
echo " [8] Agrégation des VL53"
echo "============================================================"

python3 "$D/tof_aggregate.py" \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/tof_agg.log 2>&1 &

PIDS+=($!)

sleep 2

# =====================================================================
# 9 — Nettoyage / densification ToF
# =====================================================================

echo ""
echo "============================================================"
echo " [9] Densification / nettoyage nuage ToF"
echo "============================================================"

python3 "$D/cloud_denan.py" \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/denan.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 10 — ToF -> Depth image
# =====================================================================

echo ""
echo "============================================================"
echo " [10] ToF -> Depth image"
echo "============================================================"

ros2 run rtabmap_util pointcloud_to_depthimage \
    --ros-args \
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
    -r image:=/camera/depth/image_raw \
    >/tmp/tof_depth.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 11 — ICP ODOMETRY
# =====================================================================

echo ""
echo "============================================================"
echo " [11] ICP Odometry ToF + IMU"
echo "============================================================"

ros2 run rtabmap_odom icp_odometry \
    --ros-args \
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
    -r imu:=/camera/imu \
    >/tmp/vio.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 12 — RTAB-MAP
# =====================================================================

echo ""
echo "============================================================"
echo " [12] RTAB-Map"
echo "============================================================"

ros2 run rtabmap_slam rtabmap -d \
    --ros-args \
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
    -r odom:=/odom \
    >/tmp/rtabmap.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 13 — SLAM ToF
# =====================================================================

echo ""
echo "============================================================"
echo " [13] SLAM ToF"
echo "============================================================"

python3 "$D/slam_mapper.py" \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/slam_vl53.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 14 — Dashboard RViz
# =====================================================================

echo ""
echo "============================================================"
echo " [14] Dashboard RViz"
echo "============================================================"

ros2 run dual_rviz_jazzy dual_rviz \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/slam_dashboard.log 2>&1 &

PIDS+=($!)

sleep 3

# =====================================================================
# 15 — Evaluation drift
# =====================================================================

echo ""
echo "============================================================"
echo " [15] Évaluation drift"
echo "============================================================"

python3 "$D/drift_eval.py" \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/drift.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 16 — Exploration
# =====================================================================

echo ""
echo "============================================================"
echo " [16] Exploration frontieres"
echo "============================================================"

python3 "$D/frontier_explore.py" \
    --ros-args \
    -p use_sim_time:=true \
    >/tmp/frontier.log 2>&1 &

PIDS+=($!)

# =====================================================================
# 17 — Patrol OFFBOARD — actuellement désactivé
# =====================================================================

# echo "==> [17] Patrol OFFBOARD"
#
# python3 "$D/patrol.py" \
#     --ros-args \
#     -p use_sim_time:=true \
#     >/tmp/patrol.log 2>&1 &
#
# PIDS+=($!)

# =====================================================================
# FIN
# =====================================================================

echo ""
echo "============================================================"
echo " SIMULATION PRÊTE"
echo "============================================================"
echo ""
echo "PX4 :"
echo "  commander arm -f"
echo "  commander takeoff"
echo ""
echo "Logs :"
echo "  /tmp/px4.log"
echo "  /tmp/agent.log"
echo "  /tmp/cambridge.log"
echo "  /tmp/vio.log"
echo "  /tmp/rtabmap.log"
echo "  /tmp/slam_vl53.log"
echo "  /tmp/slam_dashboard.log"
echo "  /tmp/frontier.log"
echo ""
echo "Topics :"
ros2 topic list | sort
echo ""
echo "============================================================"
echo " Console PX4 :"
echo "============================================================"
echo ""

# ---------------------------------------------------------------------
# On attend PX4.
#
# Le processus make/px4 est lancé en arrière-plan pour pouvoir démarrer
# tous les composants ROS après que Gazebo soit disponible.
# ---------------------------------------------------------------------

wait "$PX4_PID"