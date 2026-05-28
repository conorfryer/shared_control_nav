# Shared Control Nav for TurtleBot3

This package implements a shared-control navigation system for a TurtleBot3 using ROS 2 Jazzy. The robot can be driven manually, assisted by safety filtering, or automatically recovered when it reaches the edge of its operating area.

## Requirements

- ROS 2 Jazzy
- TurtleBot3
- `twist_mux`

To install `twist_mux`:

```bash
sudo apt install ros-jazzy-twist-mux
```

## Build

From the ROS 2 workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select shared_control_nav
source install/setup.bash
```

## Run

1. [Connect to and bring up the TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/).

2. Launch the shared-control system:

    ```bash
    ros2 launch shared_control_nav shared_control.launch.py
    ```

3. In a second terminal, run the custom teleop node:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run shared_control_nav shared_teleop
    ```

4. Optional: in a third terminal, run the status monitor:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run shared_control_nav status_monitor
    ```

## Controls

```text
w/x : increase/decrease linear speed
a/d : increase/decrease angular speed
s or space : stop

1 : manual mode
2 : assisted mode
3 : recovery mode

CTRL-C : quit
```

## Modes

- **Manual:** passes teleop commands through normally.
- **Assisted:** filters teleop commands using LiDAR and odometry.
- **Recovery:** drives toward the nearest recovery point, then returns to assisted mode.
