#!/usr/bin/python3
# coding=utf8
import rospy
from std_srvs.srv import SetBool
from std_msgs.msg import Int32

button_pressed = False
count_button_press = 0
count_button_release = 0

def button_callback(msg):
    global count_button_press, count_button_release, button_pressed
    # rospy.loginfo('button_state: %s'%msg.data)
    if msg.data == 0:
        count_button_release = 0
        count_button_press += 1
    else:
        count_button_release += 1
        count_button_press = 0
    if button_pressed:
        if count_button_release >= 5:
            count_button_release = 0
            count_button_press = 0
            button_pressed = False
            print('on')
    else:
        if count_button_press >= 30:
            count_button_press = 30
            button_pressed = True

rospy.init_node('button_node')
rospy.sleep(0.2)
rospy.ServiceProxy('/sensor/button/enable', SetBool)(True)
rospy.Subscriber('/sensor/button/get_button_state', Int32, button_callback)
try:
    rospy.spin()
except keyboardinterrupt:
    print("shutting down")
