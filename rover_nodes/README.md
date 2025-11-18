4 — ROS: core packages & key nodes


4.1 rover_nodes — ackermann_mapper.py
Converts high-level Ackermann steering commands or cmd_vel into individual wheel speeds and steering angles for the four corner servos.


Notes:
For precise ackermann geometry, compute inner/outer wheel angles; adapt to 4-corner steering.
You can also accept ackermann_msgs/AckermannDrive as input.


4.2 odometry_node.py
Subscribes to encoder counts and IMU to compute nav_msgs/Odometry and broadcast TF (odom -> base_link).
Key features:
subscribe chassis/encoders (Float32MultiArray from Arduino)
compute delta counts -> wheel linear displacement -> vehicle forward displacement
fuse IMU yaw for better heading
publish /odom and TF using tf2_ros.TransformBroadcaster


4.3 pid_velocity_controller.py (optional)
If you prefer PID velocity closed-loop in ROS (instead of Arduino), a node subscribes to desired wheel velocities and encoders and outputs PWM commands to the Arduino’s chassis/cmd topic. Implement per-wheel PID with configurable gains via ROS params.


4.4 joystick_teleop
Use joy + teleop_twist_joy or a custom mapping node. Example usage:
sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy
roslaunch teleop_twist_joy teleop-launch.launch
Provide a YAML mapping file to map axes/buttons to speed/steering.
