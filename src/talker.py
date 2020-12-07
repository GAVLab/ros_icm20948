#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Temperature,FluidPressure
from std_msgs.msg import Float64
import time
import board
import busio
import adafruit_mpl3115a2


def mpl3115a2_node():
     # Initialize the I2C bus.
    i2c = busio.I2C(board.SCL, board.SDA)

    # Initialize the MPL3115A2.
    sensor = adafruit_mpl3115a2.MPL3115A2(i2c)
    # Alternatively you can specify a different I2C address for the device:
    #sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)

    # You can configure the pressure at sealevel to get better altitude estimates.
    # This value has to be looked up from your local weather forecast or meteorlogical
    # reports.  It will change day by day and even hour by hour with weather
    # changes.  Remember altitude estimation from barometric pressure is not exact!
    # Set this to a value in pascals:
    sensor.sealevel_pressure = 102250

    temp_pub = rospy.Publisher('mpl3115a2/temperature', Temperature, queue_size=10)
    press_pub = rospy.Publisher('mpl3115a2/pressure', FluidPressure, queue_size=10)
    alt_pub = rospy.Publisher('mpl3115a2/altitude', Float64, queue_size=10)
    rospy.init_node('mpl3115a2')
    rate = rospy.Rate(1)
    rospy.loginfo(rospy.get_caller_id() + "  mpl3115a2 node launched.")

    while not rospy.is_shutdown():
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = 0.0
        temp_msg.variance = 0.0
        temp_pub.publish(temp_msg)
        
        press_msg = FluidPressure()
        press_msg.header.stamp = rospy.Time.now()
        press_msg.fluid_pressure = sensor.pressure
        press_msg.variance = 0.0
        press_pub.publish(press_msg)

        alt_msg = sensor.altitude
        alt_pub.publish(alt_msg)

        rate.sleep()

    rospy.loginfo(rospy.get_caller_id() + "  mpl3115a2 node finished")

if __name__ == '__main__':
    try:
        mpl3115a2_node()
    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  mpl3115a2 node exited with exception.")
