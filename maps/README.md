# Saved maps

`localization.launch.py` loads a map from here by default (`map.yaml`).

To produce it: run `slam.launch.py` and drive the robot around until the
map looks complete in RViz, then from the dev laptop (with the same
ROS_DOMAIN_ID):

```
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/mecanum_agv/maps/map
```

This writes `map.yaml` + `map.pgm` here. Rebuild (`colcon build`) so they
get installed into the package share directory, then `localization.launch.py`
will pick them up.
