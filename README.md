Ixmatix LSM9DS1 Arduino Library with ROS
======================

This is a library for LSM9DS1 (accelerometer, gyroscope and magnetometer) sensor with examples that allows you to calibrate, visualize orientation and send messages to ROS via Arduino.

It has embedded madgwick and mahony filter algorithms to get correct orientation from the sensor.

Examples
--------

* **Ixmatix_LSM9DS1_ROS** - LSM9DS1 sensor communication with ROS and use of Madgwick filter (Not compatible for Arduino Uno due to lack of memory using ROS nodehandler).
  * Gets orientation quaternion from filter algorithms.
  * Sends sensor_msgs/Imu messages.
  * Broadcasts IMU transforms.
  * [![LSM9DS1 ROS communication](https://raw.githubusercontent.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1/master/extras/imu_ros.gif)](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1)

* **Ixmatix_LSM9DS1_ROS_no_nodehandler** - The same as Ixmatix_LSM9DS1_ROS but it doesn't need ROS nodehandler and reduces memory. Check [this imu serial port node](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1_arduino_serial_ros_node) to get this arduino example to work.
  * Compatible with Arduino Uno
  
* **Ixmatix_visualise_orientation** - Use of OrientationVisualiser software for Processing to check correct orientation and configuration from sensor.
  * [![LSM9DS1 ROS communication](https://raw.githubusercontent.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1/master/extras/imu_visualiser.gif)](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1)

* **Ixmatix_calibrate_xl_gyro** - Example that allows you to get offsets for accelerometer and gyroscope.
  * [![LSM9DS1 Accelerometer and gyroscope calibration](https://raw.githubusercontent.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1/master/extras/imu_xg_calibration.jpg)](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1)

* **Ixmatics_calibrate_magnetometer** - Magnetometer calibration with Hard-Iron and Soft-Iron corrections.
  * [![LSM9DS1 Magnetometer calibration](https://raw.githubusercontent.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1/master/extras/imu_magnetometer_calibration.jpg)](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1)

  

Extras
------

* **LSM9DS1 Datasheet** - For reference.
* **MotionCal** - For magnetometer calibration.
* **OrientationVisualiser** - Script for Processing.