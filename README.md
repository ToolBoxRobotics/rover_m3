# rover_m3
Complete ROS Noetic Environment Rover - Model 3

2 — Arduino: chassis controller (Uno / Mega) — responsibilities
. talk to ROS with rosserial_arduino
. read 6 encoders, publish encoder counts / joint_states
. run closed-loop velocity PID per wheel (optional; included)
. accept motor speed commands and steering servo angles from ROS
. drive PCA9685 over I2C to produce servo PWM for steering and optional ESC/PWM for motor controllers (DRI0002) if you choose that route
. read MPU6050 (IMU) via I2C and publish sensor_msgs/Imu
. read GPS (YB-MVV21 serial) and publish sensor_msgs/NavSatFix

  Note: PCA9685 controls servos (steering). DRI0002 motor controllers accept PWM inputs — you can either:
  . have Arduino output PWM to each DRI0002 directly (use PWM pins or PCA9685 channels), and read encoder for feedback; or
  . have a ROS node talk to Arduino over rosserial and let Arduino handle low-level control. The sketch below uses rosserial commands.
