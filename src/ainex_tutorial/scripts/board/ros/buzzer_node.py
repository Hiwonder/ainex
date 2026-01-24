#!/usr/bin/python3
# coding=utf8
import time
import rospy
from ros_robot_controller.msg import BuzzerState

rospy.init_node('buzzer_node')
buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
time.sleep(0.2)
# 以1900的频率响一次(sound once in the frequency of 1900)
buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.9, repeat=1))
time.sleep(1)

while not rospy.is_shutdown():
    try:
        # 0.2s响一次(sound once every 0.2s)
        buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.1, repeat=1))
        # 延时0.1s关闭(delay 0.1s to close)
        time.sleep(0.2)
        # 1s响一次，无线循环(sound once every 1s)
        buzzer_pub.publish(BuzzerState(freq=2400, on_time=0.5, off_time=0.5, repeat=0))
        time.sleep(3)
    except KeyboardInterrupt:
        # 时间设为0就是关闭(When time is set to 0, it closes)
        buzzer_pub.publish(BuzzerState(freq=2400, on_time=0, off_time=0, repeat=1))
        break
