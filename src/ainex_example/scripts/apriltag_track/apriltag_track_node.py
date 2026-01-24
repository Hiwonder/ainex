#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# apriltag追踪(apriltag tracking)
import time
import rospy
import signal
from ainex_sdk import common, pid
from ainex_example.cfg import PIDConfig
from ainex_example.pid_track import PIDTrack
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from ainex_example.apriltag_common import Common
from apriltag_ros.msg import AprilTagDetectionArray

APRILTAG_TRACK_PID_YAML = '/home/ubuntu/ros_ws/src/ainex_example/config/apriltag_track_pid.yaml'
class ApriltagTrackNode(Common):
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
        self.head_pan_init = 500   # 左右舵机的初始值(the initial values of the left and right servos)
        self.head_tilt_init = 500  # 上下舵机的初始值(the initial values of the up and down servos)
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
        
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.apriltag_callback)
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

    def apriltag_callback(self, msg):
        if msg.detections != []:
            self.object_info = msg.detections[0].corners

    def process(self, corners):
        x = int((corners[0].x + corners[2].x)/2)
        y = int((corners[0].y + corners[2].y)/2)
        
        self.object_info = []
        if abs(x - 320) < 30:
            x = 320
        if abs(y - 240) < 30:
            y = 240
        rl_dis = self.rl_track.track(x, 320)
        ud_dis = self.ud_track.track(y, 240)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

    def run(self):
        while self.running:
            if self.start:
                if self.object_info != []:
                    self.process(self.object_info)
                if self.have_save:
                    self.dyn_client.update_configuration({'save': False})
                    common.save_yaml_data({'pid': self.config}, APRILTAG_TRACK_PID_YAML)
                    self.have_save = False

                time.sleep(0.02)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ApriltagTrackNode('apriltag_track').run()
