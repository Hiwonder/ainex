#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/19
# @author:aiden
# 手指轨迹控制(finger trajectory control)
import cv2
import time
import math
import enum
import rospy
import signal
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from ros_robot_controller.msg import BuzzerState
from ainex_sdk.common import distance, cv2_image2ros
from ainex_interfaces.msg import FingerPosition
from ainex_kinematics.gait_manager import GaitManager

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

class HandTrajectoryControlNode:
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous=True)
        self.angle = None
        self.running = True
        self.state = State.NULL
        self.points_list = []
        self.count = 0
        self.left_and_right = 0
        self.up_and_down = 0
        self.last_point = [0, 0]

        signal.signal(signal.SIGINT, self.shutdown)
        self.gait_manager = GaitManager()
        
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布(publish image processing result)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        image_sub = message_filters.Subscriber('/hand_gesture_detect/image_result', Image)
        points_sub = message_filters.Subscriber('/hand_gesture_detect/points', FingerPosition)
        
        # 同步时间戳, 时间允许有误差在0.015s(Synchronize the timestamps with allowed time difference in 0.015s)
        sync = message_filters.ApproximateTimeSynchronizer([image_sub, points_sub], 2, 0.015)
        sync.registerCallback(self.multi_callback) #执行反馈函数(execute feedback function)

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def draw_points(self, img, points, thickness=4, color=(255, 255, 0)):
        points = np.array(points).astype(dtype=np.int64)
        if len(points) > 2:
            for i, p in enumerate(points):
                if i + 1 >= len(points):
                    break
                cv2.line(img, p, points[i + 1], color, thickness)

    def multi_callback(self, ros_image, points):
        image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data).copy() # 原始 RGB 画面(original RGB image)
        gesture = points.label
        index_finger_tip = [points.points.x, points.points.y]
        if self.state != State.TRACKING:
            if gesture == "one":  # 检测食指手势， 开始指尖追踪(Detect index finger, and start fingertip tracking)
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    self.state = State.TRACKING
                    self.points_list = []
            else:
                self.count = 0
        elif self.state == State.TRACKING:
            if gesture != "two":
                if len(self.points_list) > 0:
                    last_point = self.points_list[-1]
                    if distance(last_point, index_finger_tip) < 5:
                        self.count += 1
                    else:
                        self.count = 0
                        self.points_list.extend([[int(index_finger_tip[0]), int(index_finger_tip[1])]])
                else:
                    self.points_list.extend([[int(index_finger_tip[0]), int(index_finger_tip[1])]])
            self.draw_points(image, self.points_list)
        if gesture == "five":
            left_and_right = [0]
            up_and_down = [0]
            if len(self.points_list) > 5:
                for i in self.points_list:
                    if int(i[0]) - self.last_point[0] > 0:
                        left_and_right.append(1)
                    else:
                        left_and_right.append(-1)
                    if int(i[1]) - self.last_point[1] > 0:
                        up_and_down.append(1)
                    else:
                        up_and_down.append(-1)
                    self.last_point = [int(i[0]), int(i[1])]
                self.left_and_right = sum(left_and_right)
                self.up_and_down = sum(up_and_down)
                line = cv2.fitLine(np.array(self.points_list), cv2.DIST_L2, 0, 0.01, 0.01)
                self.angle = int(abs(math.degrees(math.acos(line[0][0]))))
                self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
            
            self.state = State.NULL
            self.points_list = []
            self.draw_points(image, self.points_list)
        self.result_publisher.publish(cv2_image2ros(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), self.name))

    def run(self):
        while self.running:
            if self.angle is not None:
                print('>>>>>>', self.angle)
                if 90 >= self.angle > 60:
                    if self.up_and_down > 0:
                        print('go')
                        self.gait_manager.move(2, 0.01, 0, 0, step_num=3)
                    else:
                        print('back')
                        self.gait_manager.move(2, -0.01, 0, 0, step_num=3)
                    time.sleep(0.3)
                elif 30 > self.angle >= 0:
                    if self.left_and_right > 0:
                        print('right')
                        self.gait_manager.move(2, 0, 0.01, 0, step_num=3)
                    else:
                        print('left')
                        self.gait_manager.move(2, 0, -0.01, 0, step_num=3)
                    time.sleep(0.3)
                self.angle = None
            else:
                time.sleep(0.01)

        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandTrajectoryControlNode('hand_trajectory_control').run()
