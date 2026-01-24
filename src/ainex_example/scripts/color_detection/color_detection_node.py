#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/04
# @author:aiden
# 订阅摄像头图像，进行图像处理，发布相应的物体姿态信息和图像(subscribe to camera image, perform image processing, and publish corresponding object posture information and image)
import cv2
import time
import queue
import rospy
import signal
import numpy as np
from threading import RLock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.msg import ObjectInfo, ObjectsInfo, ColorsDetect

from ainex_sdk import common
from color_detection import ColorDetection

LAB_CONFIG_PATH = '/home/ubuntu/software/lab_tool/lab_config.yaml'
class ColorDetectionNode:
    def __init__(self, name):
        # 初始化节点(initialize node)
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name

        self.running = True
        self.image_sub = None
        self.start_detect = False
        self.image_queue = queue.Queue(maxsize=1)
        
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)

        # 获取参数(obtain parameter)
        lab_config = common.get_yaml_data(LAB_CONFIG_PATH)
        
        self.debug = rospy.get_param('~debug', False)
        self.camera = rospy.get_param('/camera')
        self.enable_roi = rospy.get_param('~enable_roi', False)
        
        # 实例化颜色识别类(instantiate color recognition class)
        self.detect = ColorDetection(lab_config['lab']['Mono'], ColorsDetect(), debug=self.enable_roi)

        # 检测图像发布(publish detected image)
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)

        # 物体位姿发布(publish object position and posture)
        self.color_info_pub = rospy.Publisher('/object/pixel_coords', ObjectsInfo, queue_size=1)

        rospy.Subscriber('~update_detect', ColorsDetect, self.update_detect, queue_size=1)
        
        rospy.Service('~enter', Empty, self.enter_func)
        rospy.Service('~exit', Empty, self.exit_func)
        rospy.Service('~start', Empty, self.start_func)
        rospy.Service('~stop', Empty, self.stop_func)
        rospy.Service('~update_lab', Empty, self.update_lab)

        self.enable_display = rospy.get_param('~enable_display', False)
        time.sleep(0.2)
        common.loginfo("%s init finish"%self.name)
        
        if self.debug:
            self.enter_func(None)
            self.start_func(None)

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            
            common.loginfo('%s shutdown'%self.name)
    
    # 开启订阅(enable subscription)
    def enter_func(self, msg):
        # 获取参数(obtain parameter)
        with self.lock:
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber('/{}/{}'.format(self.camera['camera_name'], self.camera['image_topic']), Image, self.image_callback)  # 订阅图像
        
        common.loginfo("%s enter" % self.name)
        
        return EmptyResponse()

    # 注销订阅(disable subscription)
    def exit_func(self, msg):
        with self.lock:
            self.stop_func(Empty())
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
        common.loginfo('%s exit' % self.name)

        return EmptyResponse()

    # 开启检测(start detection)
    def start_func(self, msg):
        with self.lock:
            self.start_detect = True
        
        common.loginfo("%s start" % self.name)

        return EmptyResponse()

    # 停止检测(stop detection)
    def stop_func(self, msg):
        with self.lock:
            self.start_detect = False
        
        common.loginfo("%s stop" % self.name)

        return EmptyResponse()

    # 更新颜色列表(update color list)
    def update_detect(self, msg):
        self.detect.update_detect_info(msg.data)
        
        common.loginfo('update detect')

    # 更新config参数(update config parameter)
    def update_lab(self, msg):
        config = common.get_yaml_data(LAB_CONFIG_PATH)
        self.detect.update_lab_config(config)
        
        common.loginfo('%s update lab' % self.name)

        return EmptyResponse()

    def run(self):
        while self.running:
            time_start = time.time()
            image = self.image_queue.get(block=True)
            with self.lock:
                if self.start_detect:  # 如果开启检测(if enable detection)
                    frame_result, poses = self.detect.detect(image)  # 颜色检测(color detection)
                    colors_info = []
                    for p in poses:
                        color_info = ObjectInfo()
                        color_info.label = p[0]
                        color_info.type = p[1]
                        color_info.x = p[2][0]
                        color_info.y = p[2][1]
                        color_info.width = p[3][0]
                        color_info.height = p[3][1]
                        color_info.radius = p[4]
                        color_info.angle = p[5]
                        color_info.left_point = p[6]
                        color_info.right_point = p[7]
                        colors_info.append(color_info)
                    self.color_info_pub.publish(colors_info)  # 发布位姿(publish posture)
                    time_d = 0.03 - (time.time() - time_start)
                    if time_d > 0:
                        time.sleep(time_d)
                else:
                    frame_result = image
                    time.sleep(0.03)
                ros_image = common.cv2_image2ros(frame_result, self.name)  # opencv格式转为ros(convert OpenCV to ROS)
                self.image_pub.publish(ros_image)  # 发布图像(publish image)
                if self.enable_display:
                    cv2.imshow('color_detection', frame_result)
                    key = cv2.waitKey(1)
                    if key != -1:
                        self.running = False

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式(convert the image information from ROS format to OpenCV format)

        if not self.image_queue.empty():
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self.image_queue.put_nowait(rgb_image[:, :, ::-1])
        except queue.Full:
            pass

if __name__ == '__main__':
    ColorDetectionNode('color_detection').run()
