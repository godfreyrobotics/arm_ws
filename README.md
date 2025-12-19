# arm_ws

Monorepo workspace for the modular robot arm (packages: `arm_control`, `arm_description`).

- License: GPL-3.0 (see `arm_control/LICENSE`)
- Python: requires >= 3.10

Quick start (ROS 2, colcon):

1. Install ROS 2 and dependencies
2. Build: `colcon build`
3. Source: `source install/setup.bash`
4. Run bringup: `ros2 launch arm_control bringup.launch.py`

