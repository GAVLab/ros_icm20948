# ros_icm20948

A ROS node for communicating with the [ICM20948](https://www.sparkfun.com/products/15335) 9DOF IMU.

## Description

The node communicates with the ICM20948 via i2c on the Raspberry Pi using Sparkfun's Python library: https://github.com/sparkfun/Qwiic_9DoF_IMU_ICM20948_Py.  The i2c address is preset to 0x69 which is the default.  The data is stored in the following:

* Accelerometer and Gyroscope (sensor_msgs/Imu): `icm20948/raw`
* Magnometer (sensor_msgs/MagneticField): `icm20948/mag`
* Temperature (sensor_msgs/Temperature): `icm20948/temp`
* Diagnostics (diagnostic_msgs/DiagnosticStatus): `icm20948/status`

## Installation Instructions

* Enable i2c
  ```
  sudo apt-get install i2c-tools
  i2cdetect -l
  ```
* Add `i2c-devl` to boot with `sudo nano /etc/modules-load.d/modules.conf`
* Install wiringpi `sudo apt install wiringpi`
* Connect i2c devices to Sparkfun Qwiic hat and run `i2cdetect -y 1` to identify channels
* Install driver for ICM20948 IMU: `sudo pip3 install sparkfun-qwiic-icm20948`.
* Install ahrs
  ```
  sudo apt-get update
  sudo apt-get install python3-scipy
  pip3 install ahrs
  ```

## Running the Node

* Option 1: `rosrun ros_icm20948 talker.py` 
* Option 2: `roslaunch launches/icm20948.launch`
  
## Tested Setup

It should work on other versions but Python 3 is a requirement.

* Platform: Raspberry Pi 4
* OS: Ubuntu MATE 20.04
* ROS: Noetic
* Python: 3.8.5

