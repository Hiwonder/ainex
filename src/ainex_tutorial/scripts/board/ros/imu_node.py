#!/usr/bin/python3
# coding=utf8
import time
import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    rospy.loginfo(msg)

rospy.init_node('imu_node')
time.sleep(0.2)

# 订阅imu数据(subscribe to IMU data)
rospy.Subscriber('/ros_robot_controller/imu_raw', Imu, imu_callback)
try:
    rospy.spin()
except keyboardinterrupt:
    print("shutting down")
