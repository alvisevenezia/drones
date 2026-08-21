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
#  13. Console PX4 interactive
#
# Gazebo est visible par défaut.
#
# Pour headless :
#   HEADLESS=1 ./run_slam_viz_new.sh
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

# ---------------------------------------------------------------------
# Mode GUI / HEADLESS
#
# Priorité :
#   HEADLESS=0  -> force GUI
#   HEADLESS=1  -> force headless
#   HEADLESS non défini -> détection automatique
# ---------------------------------------------------------------------

if [ -z "${HEADLESS+x}" ]; then

    if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
        HEADLESS=1
    else
        HEADLESS=0
    fi

fi

LOG_DIR="/tmp/drone_logs"
mkdir -p "$LOG_DIR"

MAIN_LOG="$LOG_DIR/simulation.log"

# Vide le log à chaque démarrage
: > "$MAIN_LOG"

PIDS=()

# FIFO utilisée pour donner stdin à PX4
PX4_STDIN_FIFO="/tmp/px4_stdin_$$"

# ---------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------

source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

if [ -f "$D/install/setup.bash" ]; then
    source "$D/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"

# ---------------------------------------------------------------------
# GUI / OpenGL
#
# Le portable n'a pas besoin de NVIDIA.
# On force Mesa llvmpipe pour RViz/Gazebo.
# ---------------------------------------------------------------------

if [ "$HEADLESS" != "1" ]; then
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
fi

# ---------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------

start_logged() {
    local name="$1"
    local logfile="$LOG_DIR/$2"
    shift 2

    (
        "$@"
    ) 2>&1 \
        | stdbuf -oL -eL \
        | sed -u "s/^/[$name] /" \
        | tee -a "$MAIN_LOG" \
        | tee "$logfile" \
        >/dev/null &

    PIDS+=("$!")
}

# ---------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------

cleanup() {
    local code=$?

    echo ""
    echo "============================================================"
    echo " Arrêt de la simulation"
    echo "============================================================"

    # Fermer proprement le FIFO
    rm -f "$PX4_STDIN_FIFO" 2>/dev/null || true

    # Arrêt propre des processus que NOUS avons lancés
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

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

WORLD_FILE="$PX4_HOME/Tools/simulation/gz/worlds/${WORLD}.sdf"

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR : monde Gazebo introuvable :"
    echo "        $WORLD_FILE"
    exit 1
fi

# Vérification dashboard
if ! ros2 pkg prefix dual_rviz_jazzy >/dev/null 2>&1; then
    echo "WARNING : dual_rviz_jazzy n'est pas disponible."
    echo "         Vérifie que $D/install/setup.bash existe."
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
# FIFO pour permettre à la console PX4 d'être interactive à la fin.
# ---------------------------------------------------------------------

rm -f "$PX4_STDIN_FIFO"
mkfifo "$PX4_STDIN_FIFO"

# ---------------------------------------------------------------------
# Important :
#
# On ne branche PAS directement stdin du terminal sur PX4 maintenant,
# car nous devons continuer le script.
#
# PX4 lit depuis le FIFO.
# À la fin, on branche le clavier sur ce FIFO.
# ---------------------------------------------------------------------

if [ "$HEADLESS" = "1" ]; then

    echo "    Mode : HEADLESS"
    echo "    Gazebo GUI : désactivé"
    echo "    RViz/Qt    : désactivé"

    (
        PX4_GZ_WORLD="$WORLD" \
        PX4_SIM_MODEL="$MODEL" \
        PX4_GZ_SIM_RENDER_ENGINE=ogre2 \
        PX4_GZ_STANDALONE=0 \
        HEADLESS=1 \
        make px4_sitl gz_x500_depth \
            < "$PX4_STDIN_FIFO" \
            2>&1\
              | stdbuf -oL -eL \
              | tee "$LOG_DIR/px4.log" \
              | tee -a "$MAIN_LOG" \
            >/dev/null &
    ) > >(tee -a /tmp/px4.log) 2>&1 &


else

    echo "    Mode : GUI"

    (
        PX4_GZ_WORLD="$WORLD" \
        PX4_SIM_MODEL="$MODEL" \
        make px4_sitl gz_x500_depth \
            < "$PX4_STDIN_FIFO" \
            2>&1
    ) > /tmp/px4.log &

fi

PX4_PID=$!
PIDS+=("$PX4_PID")

echo "    PX4 PID : $PX4_PID"

# ---------------------------------------------------------------------
# Garder un writer ouvert sur le FIFO.
#
# Sans cela, le processus PX4 peut recevoir EOF.
# ---------------------------------------------------------------------

exec 9>"$PX4_STDIN_FIFO"

# =====================================================================
# 3 — Attente PX4 / Gazebo
# =====================================================================

echo ""
echo "============================================================"
echo " [3] Attente PX4 + Gazebo"
echo "============================================================"

echo "    Attente du démarrage de Gazebo..."

sleep 5

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

wait_for_topic "/clock" 30 || true

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

CAMERA_BRIDGE_PID=$!
PIDS+=("$CAMERA_BRIDGE_PID")

sleep 3

if ! kill -0 "$CAMERA_BRIDGE_PID" 2>/dev/null; then
    echo ""
    echo "ERROR : Camera bridge arrêté immédiatement."
    echo ""
    echo "===== /tmp/cambridge.log ====="
    cat /tmp/cambridge.log
    echo "================================"
    exit 1
fi

echo "    Camera bridge actif (PID $CAMERA_BRIDGE_PID)."

echo ""
echo "    Topics ROS capteurs :"
ros2 topic list | grep -E \
    '/camera|/tof|/vl53|/imu' \
    || {
        echo "WARNING : aucun topic capteur ROS détecté."
        echo ""
        echo "===== /tmp/cambridge.log ====="
        cat /tmp/cambridge.log
        echo "================================"
    }

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
# 9 — Densification ToF
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

start_logged "ICP" "icp.log" \
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


start_logged "RTABMAP" "rtabmap.log" \
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

start_logged "SLAM_TOF" "slam_tof.log" \
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

if ros2 pkg prefix dual_rviz_jazzy >/dev/null 2>&1; then

    echo "    dual_rviz_jazzy trouvé."

    if [ "$HEADLESS" = "1" ]; then

        echo "    Dashboard désactivé en mode HEADLESS."

    else

        QT_X11_NO_MITSHM=1 \
        LIBGL_ALWAYS_SOFTWARE=1 \
        MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
        ros2 run dual_rviz_jazzy dual_rviz \
            --ros-args \
            -p use_sim_time:=true \
            >/tmp/slam_dashboard.log 2>&1 &

        PIDS+=($!)

        echo "    Dashboard RViz démarré."
        echo "    OpenGL : llvmpipe (software rendering)"

    fi

else

    echo "    WARNING : package dual_rviz_jazzy introuvable."
    echo "    Dashboard non démarré."
    echo ""
    echo "    Pour reconstruire :"
    echo "      cd $D"
    echo "      source /opt/ros/jazzy/setup.bash"
    echo "      colcon build --symlink-install --packages-select dual_rviz_jazzy"
    echo "      source install/setup.bash"

fi

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
# FIN AUTOMATIQUE
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

# =====================================================================
# 17 — CONSOLE PX4 INTERACTIVE
# =====================================================================

echo ""
echo "============================================================"
echo " Console PX4"
echo "============================================================"
echo ""
echo "La simulation est maintenant interactive."
echo "Tu peux utiliser :"
echo ""
echo "  commander arm -f"
echo "  commander takeoff"
echo ""
echo "Ctrl+C arrête toute la simulation."
echo ""
echo "============================================================"
echo ""


# ---------------------------------------------------------------------
# Lire les commandes utilisateur et les envoyer à PX4.
#
# On utilise read au lieu de cat pour conserver le comportement
# interactif du terminal.
# ---------------------------------------------------------------------

while true; do

    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo ""
        echo "PX4 s'est arrêté."
        break
    fi

    IFS= read -r line || break

    printf '%s\n' "$line" >&9

done

# ---------------------------------------------------------------------
# Attendre PX4
# ---------------------------------------------------------------------

wait "$PX4_PID" 2>/dev/null || true