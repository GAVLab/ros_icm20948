#!/usr/bin/env python3

import rospy
import time
import sys
import board
import busio
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
from adafruit_icm20x import ICM20948,AccelRange,GyroRange

def icm20948_node():

    # Initialize ROS node
    raw_pub = rospy.Publisher('icm20948/raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('icm20948/mag', MagneticField, queue_size=10)
    rospy.init_node('icm20948')
    rate = rospy.Rate(100) # frequency in Hz
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node launched.")

    i2c = busio.I2C(board.SCL, board.SDA)
    icm = ICM20948(i2c)
    icm.accelerometer_range = AccelRange.RANGE_4G
    icm.gyro_range = GyroRange.RANGE_500_DPS
    time.sleep(0.02) # short pause for good measure 
    
    while not rospy.is_shutdown():       
        raw_msg = Imu()
        raw_msg.header.stamp = rospy.Time.now()

        raw_msg.orientation.w = 0
        raw_msg.orientation.x = 0
        raw_msg.orientation.y = 0
        raw_msg.orientation.z = 0

        raw_msg.linear_acceleration.x = icm.acceleration[0] # linear acceleration (m/s^2)
        raw_msg.linear_acceleration.y = icm.acceleration[1] # linear acceleration (m/s^2)
        raw_msg.linear_acceleration.z = icm.acceleration[2] # linear acceleration (m/s^2)

        raw_msg.angular_velocity.x = icm.gyro[0] # angular velocity (rad/s)
        raw_msg.angular_velocity.y = icm.gyro[1] # angular velocity (rad/s)
        raw_msg.angular_velocity.z = icm.gyro[2] # angular velocity (rad/s)

        raw_msg.orientation_covariance[0] = -1
        raw_msg.linear_acceleration_covariance[0] = -1
        raw_msg.angular_velocity_covariance[0] = -1

        raw_pub.publish(raw_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.magnetic_field.x = icm.magnetic[0]*1e-6 # magnetic field (T)
        mag_msg.magnetic_field.y = icm.magnetic[1]*1e-6 # magnetic field (T)
        mag_msg.magnetic_field.z = icm.magnetic[2]*1e-6 # magnetic field (T)
        mag_msg.magnetic_field_covariance[0] = -1
        mag_pub.publish(mag_msg)

        rate.sleep()   
    
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
    try:
        icm20948_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node exited with exception.")
