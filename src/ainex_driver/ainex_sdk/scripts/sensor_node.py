#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/06/12
# 板载资源(onboard resource)
import os
import math
import rospy
import threading
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from ainex_sdk import button, led
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

class Sensor:
    led_path = '/home/ubuntu/share/src/.led.txt'
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name)
        self.enable_button = True
        
        rospy.Service('/sensor/button/enable', SetBool, self.set_button_enable)
        rospy.Subscriber('/sensor/led/set_led_state', Bool, self.set_led_state_callback)
        button_pub = rospy.Publisher('/sensor/button/get_button_state', Bool, queue_size=1)
        rospy.sleep(0.2)
        
        led.set_value(0)
        freq = rospy.get_param('~freq', 50)
        rate = rospy.Rate(freq)
        try:
            while not rospy.is_shutdown():
                led_value = os.popen('cat ' + self.led_path).read()
                if led_value != '':
                    led.set_value(int(led_value))
                    with open(self.led_path, 'w') as f:
                        f.write('')
                if self.enable_button:
                    button_pub.publish(button.get_button_status())
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")

    def set_button_enable(self, msg):
        self.enable_button = msg.data

        return [True, 'set_button_enable']

    def set_led_state_callback(self, msg):
        if msg.data:
            led.set_value(1)
        else:
            led.set_value(0)

if __name__ == '__main__':
    Sensor('sensor')
