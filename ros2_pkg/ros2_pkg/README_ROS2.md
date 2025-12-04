ROS2 integration notes

Files in this folder:
- `trajectory_publisher.py`: publishes a `nav_msgs/Path` on topic `trajectory`.
- `controller_node.py`: subscribes to `trajectory` and `/odom`, publishes `/cmd_vel`.

Quick run (requires ROS2 installed and sourced):

1. From a ROS2 workspace (or simply use this folder):
   - build package with ament (or use `python -m pip install -e .` for testing scripts)

2. Launch (example):
   - `ros2 launch ros2_pkg bringup_launch.py`

Notes:
- These nodes are educational shims to connect the demo to ROS2 topics. They do not implement
  the full smoothing/trajectory internals from the `src/` folder; you can replace the hard-coded
  waypoint list in `trajectory_publisher.py` with calls into the project's `trajectory` generator.
