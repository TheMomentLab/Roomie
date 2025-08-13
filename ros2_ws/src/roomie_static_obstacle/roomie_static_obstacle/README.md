# roomie_static_obstacle

No-motion tuner for static obstacle waypoint classification.

Run:

```
ros2 run roomie_static_obstacle tuner_node --ros-args -p heading_row:=A -p x_range_min:=0.3 -p x_range_max:=0.7
```

Adjust ranges in `config.py`.


ros2 run roomie_static_obstacle route_manager --ros-args -p start_wp:=C5 -p goal_wp:=A1