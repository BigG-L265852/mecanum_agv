# AMCL / localization testen zonder afgebouwde robot

Situatie (18 sep 2026): het chassis/Arduino is nog niet af, maar de RPLIDAR kan los
op de dev-laptop aangesloten worden. Deze stappen laten je toch de hele
SLAM → map opslaan → AMCL-localization pipeline testen, zonder wielodometrie.

## Waarom dit niet gewoon met `bringup.launch.py` kan

`slam_toolbox` én `AMCL` verwachten allebei een `odom → base_footprint` transform.
Normaal komt die van `mecanum_drive_node` op basis van encoder-feedback van de
Arduino. Zonder afgebouwde robot is die node niet te draaien, dus die transform
ontbreekt en niets in de TF-tree klopt.

Oplossing: `lidar_test_bringup.launch.py` publiceert een **statische, nep**
`odom → base_footprint` transform (identity, verandert nooit) in plaats van
`mecanum_drive_node`. Dat is genoeg om de node-wiring en TF-tree kloppend te
maken, maar het is **geen echte odometrie** — als je de laptop optilt en
verplaatst, registreert deze transform dat niet.

Om SLAM daar toch mee te laten werken (scan-matching in plaats van op odom
vertrouwen), gebruiken we een apart parameterprofiel:
`config/mapper_params_no_odom_test.yaml` zet `minimum_travel_distance`/
`minimum_travel_heading` op 0 (verwerk elke scan, i.p.v. wachten op
odom-beweging) en verbreedt `correlation_search_space_dimension` naar 1.5 m
(de werkelijke verplaatsing tussen scans moet binnen dit zoekvenster vallen).

Zodra de Arduino/wielodometrie werkt: gebruik gewoon `bringup.launch.py` op de
Pi en het normale `mapper_params_online_async.yaml` — dit test-profiel is dan
niet meer nodig.

## Mecanum-specifieke keuze in AMCL

`config/amcl_params.yaml` gebruikt `robot_model_type: nav2_amcl::OmniMotionModel`,
niet het standaard differential-drive model — mecanum wheels kunnen strafen/
draaien onafhankelijk van de rijrichting, en het differential-model zou de
particle cloud verkeerd verspreiden bij zijwaartse beweging.

`set_initial_pose: true` met `initial_pose` op (0, 0, 0): AMCL begint op de
oorsprong van de kaart, dat is de plek waar je begon met mappen. Zonder
startpositie publiceert AMCL geen `map → odom` en breekt Nav2 na 60 s de
opstart af. Sta je ergens anders, zet de positie dan met **2D Pose Estimate** in RViz.

## Stap 1 — map bouwen (SLAM, zonder odom)

Drie terminals op de laptop, elk met `source ~/ros2_ws/install/setup.bash`.

**Terminal 1** — LiDAR + nep-odom-transform. Kijk eerst op welke poort de LiDAR zit
(na opnieuw insteken kan `ttyUSB0` ineens `ttyUSB1` zijn):
```bash
ls /dev/ttyUSB*
ros2 launch mecanum_agv lidar_test_bringup.launch.py lidar_port:=/dev/ttyUSB0
```

**Terminal 2** — SLAM met het odom-vrije profiel:
```bash
ros2 launch mecanum_agv slam.launch.py \
  params_file:=$(ros2 pkg prefix mecanum_agv)/share/mecanum_agv/config/mapper_params_no_odom_test.yaml
```
RViz opent automatisch (met de bestaande pointcloud-workaround voor de
kapotte RViz Map-shader). Loop rustig met de laptop door de ruimte tot de
kaart compleet is.

**Terminal 3** — kaart opslaan zodra hij goed genoeg is:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/mecanum_agv/maps/map
```
Dit schrijft `maps/map.yaml` + `maps/map.pgm`. Stop daarna terminal 2 (Ctrl+C),
en rebuild zodat de map mee geïnstalleerd wordt:
```bash
cd ~/ros2_ws && colcon build --packages-select mecanum_agv
```

## Stap 2 — AMCL testen tegen de opgeslagen map

Terminal 1 (`lidar_test_bringup.launch.py`) blijft draaien. Vervang terminal 2
door:
```bash
ros2 launch mecanum_agv localization.launch.py
```
Check in RViz of `/particle_cloud` samenklontert rond je werkelijke positie.

**Let op:** omdat de odom-transform nep/statisch is, test dit alleen of AMCL
correct opstart, de map laadt, en op basis van losse scans convergeert — niet
of het écht beweging tussen updates volgt. Voor die validatie is de
wielodometrie (Arduino) nodig.

## Stap 3 — Nav2 testen (robot rijdt nog niet)

Terminal 1 (`lidar_test_bringup.launch.py`) blijft draaien. Vervang terminal 2 door:
```bash
ros2 launch mecanum_agv navigation.launch.py
```
Dit start AMCL tegen `maps/map.yaml` plus de hele Nav2-stack. Wat je kunt testen:

- **Costmaps met echte obstakels:** loop rond de LiDAR. In de *Local costmap*
  (in RViz aanzetten) verschijn je live, de *Global costmap* laat de muren zien
  met een opgeblazen rand (geel → rood).
- **Plannen:** klik met **2D Goal Pose** op de kaart. Er moet een blauwe route
  verschijnen die om obstakels heen loopt.
- **Rijcommando's:** in een extra terminal `ros2 topic echo /cmd_vel`. Na een
  klik zie je wat de robot *zou* doen (`linear.y` ≠ 0 = zijwaarts).
- **Noodrem (collision monitor):** houd een doos of je hand vlak bij de LiDAR aan
  de kant waar hij heen wil. `/cmd_vel` moet dan naar 0 gaan
  (`ros2 topic echo /collision_monitor_state`).

**Verwacht, geen bug:** omdat de nep-odometrie nooit beweegt, komt de robot
niet vooruit. Na ~10 s zonder voortgang breekt de progress checker het doel af,
probeert Nav2 een paar rondes recovery-gedrag (draaien, achteruit, wachten) en meldt
het doel na een minuut of twee als mislukt. Een nieuwe klik start opnieuw.

## Zodra de robot af is

Vervang `lidar_test_bringup.launch.py` door het normale tweetraps-model:
- Pi: `ros2 launch mecanum_agv bringup.launch.py`
- Laptop: `ros2 launch mecanum_agv slam.launch.py` (map bouwen) of
  `ros2 launch mecanum_agv navigation.launch.py` (lokaliseren + autonoom rijden),
  beide met `ROS_DOMAIN_ID` gelijk aan de Pi.
