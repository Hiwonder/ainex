#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 人脸追踪(face tracking)
import time
import rospy
import signal
from ainex_sdk import common, pid
from ainex_example.cfg import PIDConfig
from ainex_example.face_common import Common
from ainex_example.pid_track import PIDTrack
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from ainex_interfaces.msg import ObjectsInfo

FACE_TRACK_PID_YAML = '/home/ubuntu/ros_ws/src/ainex_example/config/face_track_pid.yaml'
class FaceTrackNode(Common):
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.load_param = False
        self.have_save = False
        self.object_info = []
        self.head_pan_range = [125, 875]
        self.head_tilt_range = [315, 625]
        self.head_pan_init = 500   # 左右舵机的初始值(initial value of the left-right servo)
        self.head_tilt_init = 500  # 上下舵机的初始值(initial value of the up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.config = rospy.get_param('~pid')
        self.pid_rl = pid.PID(self.config['pid1_p'], self.config['pid1_i'], self.config['pid1_d'])
        self.pid_ud = pid.PID(self.config['pid2_p'], self.config['pid2_i'], self.config['pid2_d'])
        
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        Server(PIDConfig, self.dynamic_reconfigure_callback)
        self.load_param = True
        self.dyn_client = Client(self.name, timeout=10)
        self.dyn_client.update_configuration(self.config)
        
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_face_callback)
        self.motion_manager.run_action('stand')

        if rospy.get_param('~start', True):
            self.enter_func(None)
            self.start_srv_callback(None)
            common.loginfo('start track')

    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    def dynamic_reconfigure_callback(self, config, level):
        if self.load_param:
            self.rl_track.update_pid(pid.PID(config['pid1_p'], config['pid1_i'], config['pid1_d']))
            self.ud_track.update_pid(pid.PID(config['pid2_p'], config['pid2_i'], config['pid2_d']))
            self.config['pid1_p'] = config['pid1_p']
            self.config['pid1_i'] = config['pid1_i']
            self.config['pid1_d'] = config['pid1_d']
            self.config['pid2_p'] = config['pid2_p']
            self.config['pid2_i'] = config['pid2_i']
            self.config['pid2_d'] = config['pid2_d']
            if config['save']:
                self.have_save = True
        
        return config

    def get_face_callback(self, msg):
        if msg.data != []:
            self.object_info = msg.data[0]

    def process(self, center):
        if abs(center.x - center.width/2) < 20:
            center.x = center.width/2
        if abs(center.y - center.height/2) < 20:
            center.y = center.height/2
        rl_dis = self.rl_track.track(center.x, center.width/2)
        ud_dis = self.ud_track.track(center.y, center.height/2)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

    def run(self):
        while self.running:
            if self.start:
                if self.object_info != []:
                    self.process(self.object_info)
                    self.object_info = []
                if self.have_save:
                    self.dyn_client.update_configuration({'save': False})
                    common.save_yaml_data({'pid': self.config}, FACE_TRACK_PID_YAML)
                    self.have_save = False
                time.sleep(0.02)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    FaceTrackNode('face_track').run()
