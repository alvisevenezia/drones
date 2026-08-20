# Architecture de la simulation drone SLAM

Pile : **ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL** (monde *Depot*), capteur **VL53L9CX** (dToF), odométrie **ICP-inertielle** (rtabmap), éval de drift vs vérité PX4.

---

## 1. Lancement

```bash
./run_slam_viz.sh                 # lance TOUT (sim + capteurs + SLAM + RViz)
# dans la console pxh> :  commander arm -f   puis   commander takeoff
# (option) exploration autonome, terminal 2 :
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/Documents/drones/patrol.py --ros-args -p use_sim_time:=true
```

---

## 2. Fichiers du projet — `~/Documents/drones/`

| Fichier | Rôle |
|---|---|
| `run_slam_viz.sh` | **Lanceur principal** : orchestre agent, pont, TF, odométrie, rtabmap, mapper, RViz×2, drift-eval, PX4+Gazebo |
| `run_camera_bridge.sh` | Pont **gz → ROS 2** (RGB, IMU, nuage ToF, clock) |
| `cloud_denan.py` | Filtre les NaN du nuage ToF (`/tof/points` → `/tof/points_dense`) — le depth_camera sort un nuage organisé avec invalides |
| `slam_mapper.py` | **Carte A** : accumule le nuage ToF dans le repère `odom` → `/slam_map_tof` |
| `drift_eval.py` | Compare odométrie `/odom` vs **vérité PX4** → `/vio_drift` + flèches `/vio_marker` (bleu) et `/truth_marker` (rouge) |
| `patrol.py` | **Exploration autonome** OFFBOARD (créneaux + scans lacet + évitement ToF) |
| `px4_odom.py` | Odométrie de repli = pose EKF2 PX4 (4-DOF), NON active par défaut |
| `pose_relay.py` | Pose PX4 → `/drone/pose` pour le cube Unity |
| `check_vl53.py` | Debug : compte les points du nuage ToF |
| `fastlio_vl53l9cx.yaml` | Config FAST-LIO (**option**, non active — voir §6) |
| `slam.rviz` / `slam_cam.rviz` | Configs RViz historiques (remplacées par le dashboard Qt) |
| `dual_rviz_jazzy/` | App Qt unique : telemetry gauche + reconstruction 3D + carte/goals droite |
| `ARCHITECTURE.md` | Ce document |

---

## 3. Fichiers hors projet

| Emplacement | Rôle |
|---|---|
| `~/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf` | **Capteurs du drone** : caméra RGB `IMX214`, IMU `camera_imu` (200 Hz), **VL53L9CX** (depth_camera 54×42, modèle datasheet leogue) |
| `~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf` | Drone x500, **inclut** OakD-Lite |
| `~/PX4-Autopilot/Tools/simulation/gz/worlds/tugbot_depot.sdf` | Monde **Depot** (entrepôt tugbot) |
| `~/ros2_ws/` | Workspace ROS : `px4_msgs`, `ros_gz`, MicroXRCE, **`FAST_LIO_ROS2`** (option) |
| `~/ros2_ws/src/FAST_LIO_ROS2/` | FAST-LIO (compilé, dép. Livox retirée) — config `fastlio_vl53l9cx.yaml` |
| `~/DroneSim/` | Projet Unity (cube suiveur, pont ROS-TCP) |
| `/tmp/*.log` | Logs de chaque nœud (`vio.log`, `rtabmap.log`, `denan.log`, `drift.log`, `cambridge.log`, `agent.log`…) |

---

## 4. Chaîne des nœuds (lancés par `run_slam_viz.sh`)

```
[1] MicroXRCEAgent            PX4 <-> ROS 2 (uXRCE-DDS, port 8888)
[2] run_camera_bridge.sh      capteurs gz -> ROS 2
[3] static_tf                 base_link -> camera_link (optique)
[3b] static_tf                camera_link -> tof_link (FLU, x-avant)
[3c] (desactive)              projection ToF->depth (plus utilisee)
[3d] cloud_denan.py           /tof/points -> /tof/points_dense (sans NaN)
[4] icp_odometry              /tof/points_dense + /camera/imu -> /odom (+TF odom->base_link)
[5] rtabmap                   carte dense + loop-closure (nuage ToF + RGB)
[6] slam_mapper.py            carte A -> /slam_map_tof
[7] dual_rviz_jazzy/dual_rviz Dashboard Qt unique (caméras + vues SLAM + grille + stats)
[7b] drift_eval.py            drift /odom vs verite PX4
[8] PX4 SITL + Gazebo         make px4_sitl gz_x500_depth (monde tugbot_depot, HEADLESS)
```

Pipeline SLAM :
```
VL53L9CX (depth_camera 54x42) --/tof/points--> cloud_denan --/tof/points_dense-->  icp_odometry --> /odom
   caméra RGB + IMU 200 Hz -------------------------------------------------------> (icp fuse l'IMU)
   rtabmap (RGB loop-closure + nuage) --> /cloud_map ;  slam_mapper --> /slam_map_tof
   drift_eval : /odom vs /fmu/out/vehicle_local_position_v1 --> /vio_drift + marqueurs
```

---

## 5. Topics principaux

| Topic | Type | Source |
|---|---|---|
| `/camera/rgb/image_raw`, `/camera/rgb/camera_info` | Image / CameraInfo | pont (caméra IMX214) |
| `/camera/imu` | Imu (200 Hz) | pont (camera_imu) |
| `/tof/points` | PointCloud2 (organisé 54×42) | pont (VL53L9CX depth_camera) |
| `/tof/points_dense` | PointCloud2 (dense) | `cloud_denan.py` |
| `/odom` | Odometry | `icp_odometry` |
| `/slam_map_tof` | PointCloud2 | `slam_mapper.py` (carte A) |
| `/cloud_map` | PointCloud2 | `rtabmap` (carte B dense) |
| `/vio_drift` | Float64 (m) | `drift_eval.py` |
| `/vio_marker` (bleu) / `/truth_marker` (rouge) | Marker | `drift_eval.py` |
| `/fmu/out/vehicle_local_position_v1` | VehicleLocalPosition | PX4 (vérité terrain) |

**Arbre TF** : `map` (rtabmap) → `odom` (icp) → `base_link` → `camera_link` → `tof_link`.

---

## 6. Odométrie : ICP (actif) vs FAST-LIO (option)

- **ICP-inertiel (`icp_odometry`, rtabmap) = ACTIF** — bloc [4] de `run_slam_viz.sh`. Recale les nuages ToF + fusionne l'IMU. ~5 % de drift, cartographie les étagères. Adapté au ToF avant épars.
- **FAST-LIO = repli commenté** dans [4]. Compilé (`~/ros2_ws/src/FAST_LIO_ROS2`, dép. Livox retirée), mais son `pcl::fromROSMsg` ingère mal le nuage ToF épars (« No data to copy / Too few points ») — FAST-LIO vise des LiDAR denses. À reprendre si besoin d'un vol très dynamique.

---

## 7. Pièges / notes

- **Repère du nuage ToF = `tof_link` (FLU, x-avant)**, pas `camera_link` (optique) — sinon axes inversés (montée → avant).
- Le **depth_camera sort des NaN** → `cloud_denan` obligatoire avant ICP/FAST-LIO.
- Lire les topics capteurs en CLI : ajouter `--qos-profile sensor_data` (best-effort). Les affichages RViz sont en **Best Effort**.
- Décollage doux si l'odométrie décroche : `param set MPC_TKO_SPEED 0.7` dans `pxh>`.
- `drift_eval` publie les flèches depuis le **topic `/odom`** (robuste), pas via la TF.
- Repo **pas sous git** — des `.bak` existent pour `slam_mapper.py`, `run_slam_viz.sh`, `model.sdf`.

---

## 8. Reste à faire (optionnel)

- Porter le **bruit datasheet** du VL53L9CX (leogue `bridge.py` : σ 1 mm + 0,07 %, dropouts) en nœud ROS.
- Nettoyer le filtre `fx>0` de `slam_mapper` (carte A verte, secondaire).
- Reprendre FAST-LIO si vol très dynamique requis (régler le format de nuage attendu par son PCL).

---

## 9. Fusion EKF IMU + ICP (robot_localization)

**But** : mieux exploiter l'IMU (200 Hz) pour suivre les mouvements rapides (montée, virages) que l'ICP seul (~5-10 Hz) rate → moins de drift dynamique + odométrie lissée.

**Installation** (une fois) : `sudo apt install ros-jazzy-robot-localization`.

**Comment ça marche** — un filtre de Kalman étendu (EKF) combine deux sources complémentaires :

| Source | Qualité | Défaut |
|---|---|---|
| **ICP** (`/icp_odom`) | position + cap justes, sans biais long-terme | **lent** (~5-10 Hz) → rate les mouvements rapides |
| **IMU** (`/camera/imu`) | **rapide** (200 Hz), accéleration + vitesse angulaire | dérive vite si intégrée seule |

L'EKF alterne deux étapes :
1. **Prédiction** (à 50 Hz) : il **intègre l'IMU** (accéléro + gyro) pour propager la pose entre deux scans ICP → l'état suit les mouvements rapides en temps réel.
2. **Correction** : quand une pose ICP arrive, il **recale** la position/le cap dessus → pas de dérive long-terme de l'IMU.

Résultat : **`/odom` devient lisse, haute fréquence, et robuste aux mouvements rapides**, tout en restant ancré à l'ICP.

**Câblage** (`run_slam_viz.sh`) :
```
icp_odometry  --/icp_odom-->  ekf_node  --/odom--> (rtabmap, frontier, patrol, drift_eval)
/camera/imu (200 Hz) ------->  ekf_node  --TF odom->base_link-->
```
- `icp_odometry` [4] : `publish_tf:=false` + sortie renommée `/icp_odom` (il ne publie plus la TF ni `/odom`).
- `ekf_node` [4b] : lit `ekf.yaml`, fusionne `/icp_odom` + `/camera/imu`, publie `/odom` (remap de `/odometry/filtered`) **et** la TF `odom->base_link`.
- Tout l'aval (`rtabmap`, `frontier_explore`, `patrol`, `drift_eval`) consomme `/odom` **inchangé** — c'est juste devenu la sortie EKF.

**Config** : `ekf.yaml` — `odom0=/icp_odom` (x,y,z,yaw), `imu0=/camera/imu` (gyro + accéléro), `two_d_mode=false`, `publish_tf=true`. Ajuster `process_noise_covariance` pour doser la confiance IMU vs ICP.
