#!/usr/bin/python3
# coding=utf8
import time
import rospy
from ros_robot_controller.msg import RGBState, RGBsState

rospy.init_node('rgb_node')
rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
time.sleep(0.2)
# 设置rgb颜色(set RGB color)
rgb_pub.publish(RGBsState([RGBState(1, 255, 0, 0)]))
time.sleep(1)
rgb_pub.publish(RGBsState([RGBState(1, 0, 255, 0)]))
time.sleep(1)
rgb_pub.publish(RGBsState([RGBState(1, 0, 0, 255)]))
time.sleep(1)
rgb_pub.publish(RGBsState([RGBState(1, 0, 0, 0)]))
