#!/usr/bin/env bash
# =====================================================================
#  Pile drone SLAM — étapes PRIVILÉGIÉES (sudo)
#  Généré par Claude Code.
#  Lance-le UNE fois :   bash ~/Documents/drones/install_sudo.sh
#  sudo te demandera ton mot de passe une seule fois (mis en cache).
# =====================================================================
set -uo pipefail

echo "==> Authentification sudo (mot de passe demandé une fois)"
sudo -v || { echo "Echec sudo, abandon."; exit 1; }
# garde le ticket sudo actif pendant tout le script
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) &
SUDO_KEEPALIVE=$!
trap 'kill "$SUDO_KEEPALIVE" 2>/dev/null' EXIT

# --- 1/4 : rosdep --------------------------------------------------
echo ""
echo "==> [1/4] rosdep init + update"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
else
  echo "    rosdep déjà initialisé, on saute init."
fi
rosdep update

# --- 2/4 : Gazebo Harmonic + pont ros_gz (étape 2) -----------------
echo ""
echo "==> [2/4] Gazebo Harmonic + ros_gz"
sudo apt update
sudo apt install -y ros-jazzy-ros-gz

# --- 3/4 : dépendances de build PX4 (étape 3) ----------------------
echo ""
echo "==> [3/4] Dépendances PX4 (setup/ubuntu.sh, sans toolchain embarqué)"
if [ -d "$HOME/PX4-Autopilot" ] && [ -f "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" ]; then
  bash "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx \
    || bash "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh"
else
  echo "    !! PX4-Autopilot pas encore prêt — relance ce bloc plus tard."
fi

# --- 4/4 : installation Micro-XRCE-DDS Agent (étape 4a) ------------
echo ""
echo "==> [4/4] Installation Micro-XRCE-DDS Agent"
if [ -f "$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent" ]; then
  ( cd "$HOME/Micro-XRCE-DDS-Agent/build" && sudo make install && sudo ldconfig /usr/local/lib/ )
else
  echo "    !! build Micro-XRCE pas prêt — relance ce bloc plus tard."
fi

echo ""
echo "====================================================="
echo "  ÉTAPES SUDO TERMINÉES."
echo "  Reviens dire à Claude Code : « script sudo fait »."
echo "====================================================="
