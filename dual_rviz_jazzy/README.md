# dual_rviz_jazzy

Application Qt/RViz2 pour centraliser le debug SLAM dans une seule fenêtre.

- colonne gauche: caméra RGB, caméra depth, vitesse, drift et infos RTAB-Map
- panneau central: reconstruction 3D (`/slam_map_tof` + `/cloud_map`)
- panneau droit: grille d'exploration, frontières, goal marker et marqueurs drift
- le panneau 3D central propose un bouton pour basculer entre nuage en points colorés et cubes orange

L'app charge deux configurations RViz séparées pour éviter la superposition des
displays et garder les vues visuellement distinctes.

## Compilation

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/drones
colcon build --packages-select dual_rviz_jazzy
source install/setup.bash
ros2 run dual_rviz_jazzy dual_rviz
```
