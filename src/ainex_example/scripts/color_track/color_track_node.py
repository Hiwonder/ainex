#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 颜色追踪(color tracking)
import time
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common, pid
from ainex_example.cfg import PIDConfig
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

COLOR_TRACK_PID_YAML = '/home/ubuntu/ros_ws/src/ainex_example/config/color_track_pid.yaml'
class ColorTrackNode(Common):
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.load_param = False
        self.have_save = False
        self.object_info = []
        self.image_process_size = [160, 120]
        self.head_pan_range = [125, 875]
        self.head_tilt_range = [315, 625]
        self.head_pan_init = 500   # 左右舵机的初始值(initial values of left and right servos)
        self.head_tilt_init = 500  # 上下舵机的初始值(initial values of up and down servos)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.config = rospy.get_param('~color_track')
        self.pid_rl = pid.PID(self.config['pid1_p'], self.config['pid1_i'], self.config['pid1_d'])
        self.pid_ud = pid.PID(self.config['pid2_p'], self.config['pid2_i'], self.config['pid2_d'])
        
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        Server(PIDConfig, self.dynamic_reconfigure_callback)
        self.load_param = True
        self.dyn_client = Client(self.name, timeout=10)
        self.dyn_client.update_configuration(self.config)
        
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        self.motion_manager.run_action('stand')

        if rospy.get_param('~start', True):
            target_color = rospy.get_param('~color', 'blue')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))
            self.start_srv_callback(None)
            common.loginfo('start track %s' % target_color)

    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    def set_color_srv_callback(self, msg):
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'circle'
        param.image_process_size = self.image_process_size
        param.min_area = 10
        param.max_area = self.image_process_size[0]*self.image_process_size[1]
        self.detect_pub.publish([param])

        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

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

    def get_color_callback(self, msg):
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
                    common.save_yaml_data({'pid': self.config}, COLOR_TRACK_PID_YAML)
                    self.have_save = False
                time.sleep(0.02)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorTrackNode('color_track').run()
