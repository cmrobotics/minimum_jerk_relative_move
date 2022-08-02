# Shelf Detector Tests

## Run tests

- Run: `colcon test --packages-select shelf_detector && cat log/latest/shelf_detector/stdout.log`

## Shelf Detection

- Launch either:
    - Serena: `ros2 launch serena_bringup hardware_launch_no_namespace.py`
    - Simulation: `ros2 launch robot_bringup bringup_launch.py`

- Open RVIZ:
    - Serena: `ros2 launch serena_bringup rviz_launch_no_namespace.py`
    - Simulation: `ros2 launch robot_bringup rviz_launch.py`

- Activate detection:
    - Serena & Simulation: `ros2 service call /activate_shelf_detection cmr_msgs/srv/ActivateShelfDetection "{}"`

- Shelf:
    - In Gazebo, spawn the trolley description in the robot FOV.
    - In the real word, move the shelf in serena's FOV.

Two frames should appear to represent the shelf center position. If you move the shelf around, the frame on RVIZ should move as well.

- Deactivate detection:
    - Serena & Simulation: `ros2 service call /deactivate_shelf_detection cmr_msgs/srv/DeactivateShelfDetection "{}"`

The frames should not disappear. Computation will however stopped and the last cached pose will be used. It prevents jitters in the shelf detection due to AMCL's localization when the robot is very close to the shelf.
