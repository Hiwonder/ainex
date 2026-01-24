#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/10
# @author:aiden
# 玩法的通用框架(game's common frame)
import time
import rospy
from threading import RLock
from ainex_sdk import common
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.msg import ColorsDetect
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

class Common:
    def __init__(self, name, head_pan_init, head_tilt_init):
        self.name = name
        self.start = False
        self.head_pan_init = head_pan_init    # 左右舵机的初始值(initial values of left and right servos)
        self.head_tilt_init = head_tilt_init  # 上下舵机的初始值(initial values of up and down servos)

        self.lock = RLock()
        
        # 机器人行走的库调用(call robot walking library)
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        
        # 更新颜色识别参数(update color recognition parameter)
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)

        rospy.Service('~enter', Empty, self.enter_func)  # 进入玩法(enter game)
        rospy.Service('~exit', Empty, self.exit_func)  # 退出玩法(exit game)
        rospy.Service('~start', Empty, self.start_srv_callback)  # 开始玩法(start game)
        rospy.Service('~stop', Empty, self.stop_srv_callback)  # 停止玩法(stop game)

        time.sleep(0.2)
        rospy.set_param('~init_finish', True)
        common.loginfo('%s init_finish' % self.name)  

    def init_action(self, head_pan_init, head_tilt_init, delay=200):
        # 初始位置(initial position)
        self.motion_manager.set_servos_position(delay, [[23, head_pan_init], [24, head_tilt_init]])
        self.gait_manager.stop()
        time.sleep(delay/1000)

    def enter_func(self, msg):
        self.init_action(self.head_pan_init, self.head_tilt_init)
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        common.loginfo("%s enter" % self.name)
        return EmptyResponse()

    def exit_func(self, msg):
        self.stop_srv_callback(Empty())
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        common.loginfo('%s exit' % self.name)
        return EmptyResponse()

    def start_srv_callback(self, msg):
        with self.lock: 
            rospy.ServiceProxy('/color_detection/start', Empty)()
            self.start = True
        common.loginfo('%s start' % self.name)
        return EmptyResponse()

    def stop_srv_callback(self, msg):
        with self.lock:
            rospy.ServiceProxy('/color_detection/stop', Empty)()
            self.start = False
            self.detect_pub.publish(ColorsDetect())
            self.init_action(self.head_pan_init, self.head_tilt_init)
        common.loginfo('%s stop' % self.name)
        return EmptyResponse()
