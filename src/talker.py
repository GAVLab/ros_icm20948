#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Temperature,MagneticField,Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticStatus
import time
import qwiic_icm20948
import sys

def icm20948_node():

    # Initialize ROS node
    raw_pub = rospy.Publisher('icm20948/raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('icm20948/mag', MagneticField, queue_size=10)
    temp_pub = rospy.Publisher('icm20948/temp', Temperature, queue_size=10)
    status_pub = rospy.Publisher('icm20948/status', DiagnosticStatus, queue_size=10)
    rospy.init_node('icm20948')
    rate = rospy.Rate(20) # frequency in Hz
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node launched.")

    IMU = qwiic_icm20948.QwiicIcm20948()

    while IMU.connected == False:
        message = "The Qwiic ICM20948 device isn't connected to the system. Please check your connection"
        rospy.loginfo(message)
        status_msg = DiagnosticStatus()
        status_msg.level = 2
        status_msg.name = "icm20948 IMU"
        status_msg.message = message
        status_pub.publish(status_msg)        

    IMU.begin()    

    while not rospy.is_shutdown():
        
        if IMU.dataReady():
            IMU.getAgmt() # read data
            raw_msg = Imu()
            raw_msg.header.stamp = rospy.Time.now()

            raw_msg.orientation.w = 0
            raw_msg.orientation.x = 0
            raw_msg.orientation.y = 0
            raw_msg.orientation.z = 0

            raw_msg.linear_acceleration.x = IMU.axRaw
            raw_msg.linear_acceleration.y = IMU.ayRaw
            raw_msg.linear_acceleration.z = IMU.azRaw

            raw_msg.angular_velocity.x = IMU.gxRaw
            raw_msg.angular_velocity.y = IMU.gyRaw
            raw_msg.angular_velocity.z = IMU.gzRaw

            raw_msg.orientation_covariance[0] = -1
            raw_msg.linear_acceleration_covariance[0] = -1
            raw_msg.angular_velocity_covariance[0] = -1

            raw_pub.publish(raw_msg)

            mag_msg = MagneticField()
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.magnetic_field.x = IMU.mxRaw
            mag_msg.magnetic_field.y = IMU.myRaw
            mag_msg.magnetic_field.z = IMU.mzRaw
            mag_msg.magnetic_field_covariance[0] = -1
            mag_pub.publish(mag_msg)

            temp_msg = Temperature()
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.temperature = IMU.tmpRaw/100
            temp_msg.variance = 0.0
            temp_pub.publish(temp_msg)
            
            status_msg = DiagnosticStatus()
            status_msg.level = 0
            status_msg.name = "icm20948 IMU"
            status_msg.message = ""
            status_pub.publish(status_msg)

            rate.sleep()   
    
    rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
    try:
        icm20948_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  mpl3115a2 node exited with exception.")
