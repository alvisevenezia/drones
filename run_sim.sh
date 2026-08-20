#!/usr/bin/env bash
# =====================================================================
#  Lancement quotidien — simulation drone SLAM (Gazebo + PX4 + ROS 2)
#  Usage :  bash ~/Documents/drones/run_sim.sh
#  Ouvre l'agent uXRCE-DDS (en fond) puis PX4 SITL + Gazebo (fenetre).
#  Ctrl+C dans PX4 (ou taper 'shutdown' dans pxh>) pour tout arreter.
# =====================================================================
set -uo pipefail

echo "==> [1/2] Micro-XRCE-DDS Agent (pont PX4 <-> ROS 2, port 8888)"
if pgrep -x MicroXRCEAgent >/dev/null; then
  echo "    deja lance, on reutilise."
else
  MicroXRCEAgent udp4 -p 8888 >/tmp/microxrce_agent.log 2>&1 &
  echo "    agent demarre (log: /tmp/microxrce_agent.log)"
fi

sleep 1
echo ""
echo "==> [2/2] PX4 SITL + Gazebo Harmonic (monde Depot, fenetre graphique)"
echo "    Dans la console pxh> :  commander arm -f   puis   commander takeoff   (land pour reposer)"
echo ""
cd "$HOME/PX4-Autopilot" && PX4_GZ_WORLD=tugbot_depot make px4_sitl gz_x500_depth

echo ""
echo "==> PX4 arrete. Arret de l'agent."
pkill -x MicroXRCEAgent 2>/dev/null || true
