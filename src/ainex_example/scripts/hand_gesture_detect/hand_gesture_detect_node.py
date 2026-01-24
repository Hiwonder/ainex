#!/usr/bin/env python3
# encoding: utf-8
# 手势检测(gesture control)
import cv2
import time
import rospy
import numpy as np
import mediapipe as mp
import ainex_sdk.fps as fps
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from ainex_interfaces.msg import FingerPosition, PixelPosition
from ainex_sdk.common import vector_2d_angle, distance, cv2_image2ros

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert landmarks from normalized output of Mediapipe to pixel coordinates)
    :param img: 像素坐标对应的图片(image corresponding to pixel coordinates)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度(calculate the bending angle of each finger)
    :param landmarks: 手部关键点(the key points of the hand)
    :return: 各个手指的角度(each finger's angle)
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list

def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势(Determine the gesture made by the fingers based on the two-dimensional features)
    :param angle_list: 各个手指弯曲的角度(the angles of each finger's bending)
    :return : 手势名称字符串(gesture name string)
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str

class HandGestureDetectNode:
    def __init__(self, name):
        rospy.init_node(name)  # launch里的name会覆盖此处的name，所以要修改name，需要修改launch里的name, 为了一致性此处name会和launch name保持一致(The name in "launch" will override the name here. If you want to modify the name, you need to modify the name in "launch". The name here should be kept consistent with the name in launch for consistency)

        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.start = False
        self.running = True
        self.image = None
        self.fps = fps.FPS()  # fps计算器(FPS calculator)

        self.camera = rospy.get_param('/camera')
        rospy.Subscriber('/{}/{}'.format(self.camera['camera_name'], self.camera['image_topic']), Image, self.image_callback)  # 订阅图像(subscribe to image)  # 摄像头订阅(subscribe to camera)

        self.point_publisher = rospy.Publisher('~points', FingerPosition, queue_size=1)  # 使用~可以自动加上前缀名称(use ~ to automatically add prefix name)
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布(publish image processing result)
        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法(enter game)
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法(exit game)
        if rospy.get_param('~start'):
            self.start_srv_callback(None)
        
        rospy.set_param('~init_finish', True)
    
    def start_srv_callback(self, msg):
        rospy.loginfo('start hand trajectory')
        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop hand trajectory')
        self.start = False

        return TriggerResponse(success=True)

    def run(self):
        points_list = []
        while self.running:
            if self.image is not None:
                image_flip = cv2.flip(self.image, 1)
                self.image = None
                bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
                gesture = "none"
                index_finger_tip = [0, 0]
                if self.start:
                    try:
                        results = self.hand_detector.process(image_flip)
                        if results is not None and results.multi_hand_landmarks:
                            for hand_landmarks in results.multi_hand_landmarks:
                                self.drawing.draw_landmarks(
                                    bgr_image,
                                    hand_landmarks,
                                    mp.solutions.hands.HAND_CONNECTIONS)
                                landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                                angle_list = (hand_angle(landmarks))
                                gesture = (h_gesture(angle_list))
                                index_finger_tip = landmarks[8].tolist()

                    except Exception as e:
                        print(e)
                # else:
                    # time.sleep(0.01)
                msg = FingerPosition()
                msg.header = Header(stamp=rospy.Time.now())
                msg.label = gesture
                msg.points.x = index_finger_tip[0]
                msg.points.y = index_finger_tip[1]
                self.point_publisher.publish(msg)
                self.result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), self.name))

    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)

if __name__ == "__main__":
    HandGestureDetectNode('hand_gesture_detect').run()
