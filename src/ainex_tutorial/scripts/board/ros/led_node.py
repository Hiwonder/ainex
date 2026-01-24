#!/usr/bin/python3
# coding=utf8
import time
import rospy
from std_msgs.msg import Bool

rospy.init_node('led_node')
led_pub = rospy.Publisher('/sensor/led/set_led_state', Bool, queue_size=1)
time.sleep(0.2)
# 打开led(turn on LED)
led_pub.publish(True)
# 延时0.5s关闭(delay 0.5s tp close)
time.sleep(0.5)
led_pub.publish(False)
