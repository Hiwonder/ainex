#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/19
# @author:aiden
# 手势控制(gesture control)
import time
import rospy
import signal
import numpy as np
from ros_robot_controller.msg import BuzzerState
from ainex_interfaces.msg import FingerPosition
from ainex_kinematics.motion_manager import MotionManager

class HandGestureControlNode:
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous=True)
        self.running = True
        self.gesture = None
        self.gesture_list = [0, 0, 0, 0, 0]
        self.action_finish = True
        self.count = 0

        signal.signal(signal.SIGINT, self.shutdown)
        rospy.Subscriber('/hand_gesture_detect/points', FingerPosition, self.get_hand_points_callback)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.motion_manager = MotionManager()
        self.motion_manager.run_action('stand')
    
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_hand_points_callback(self, msg):
        if self.action_finish:
            if msg.label == 'one':
                self.gesture_list[0] += 1
            elif msg.label == 'two':
                self.gesture_list[1] += 1
            elif msg.label == 'three':
                self.gesture_list[2] += 1
            elif msg.label == 'four':
                self.gesture_list[3] += 1
            elif msg.label == 'five':
                self.gesture_list[4] += 1

            self.count += 1
            if self.count > 2:
                self.count = 0
                gesture_list  = np.array(self.gesture_list)
                self.gesture_list = [0, 0, 0, 0, 0]
                if np.amax(gesture_list) > 1:
                    self.gesture = np.argmax(gesture_list) + 1
                    self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
                    self.action_finish = False

    def run(self):
        while self.running:
            if self.gesture is not None:
                print('>>>>>>', self.gesture)
                if self.gesture == 1:
                    self.motion_manager.run_action('greet')
                elif self.gesture == 2:
                    self.motion_manager.run_action('twist')
                # elif self.gesture == 3:
                    # self.motion_manager.run_action('three')
                # elif self.gesture == 4:
                    # self.motion_manager.run_action('four')
                # elif self.gesture == 5:
                    # self.motion_manager.run_action('twist')
                self.gesture = None
                time.sleep(0.5)
                self.action_finish = True
            else:
                time.sleep(0.01)

        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandGestureControlNode('hand_gesture_control').run()
