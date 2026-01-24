#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/06
# @author:aiden
# 巡线逻辑处理(line following logic processing)
import math
from ainex_sdk import misc, common

class VisualPatrol:
    def __init__(self, gait_manager):
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        self.x_range = [0, 0.010]  # 前进脚步大小单位m(the step length for walking forward in units of m)
        self.yaw_range = [-8, 8]  # 转弯范围正负8度(the turning range is between -8 and 8)
        self.gait_manager = gait_manager
        self.go_gait_param = self.gait_manager.get_gait_param()
        self.go_gait_param['body_height'] = 0.025
        self.go_gait_param['step_height'] = 0.015
        self.go_gait_param['hip_pitch_offset'] = 15
        self.go_gait_param['z_swap_amplitude'] = 0.006
        self.go_dsp = [300, 0.2, 0.02]
        self.go_arm_swap = 30

        self.turn_gait_param = self.gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.025
        self.turn_gait_param['step_height'] = 0.02
        self.turn_gait_param['hip_pitch_offset'] = 15
        self.turn_gait_param['z_swap_amplitude'] = 0.006
        self.turn_dsp = [400, 0.2, 0.02]
        self.turn_arm_swap = 30

        self.x_max = self.x_range[1]

    def update_turn_gait(self, dsp=None, x_max=None, arm_swap=None, walking_param=None):
        if walking_param is not None:
            self.turn_gait_param = walking_param
        if dsp is not None:
            self.turn_dsp = dsp
        if x_max is not None:
            self.x_max = x_max
        if arm_swap is not None:
            self.turn_arm_swap = arm_swap

    def update_go_gait(self, dsp=None, x_max=None, arm_swap=None, walking_param=None):
        if walking_param is not None:
            self.go_gait_param = walking_param
        if dsp is not None:
            self.go_dsp = dsp
        if x_max is not None:
            self.x_max = x_max
        if arm_swap is not None:
            self.go_arm_swap = arm_swap

    def process(self, x, width):
        # 左右根据x坐标值进行调整(adjust left and right based on x-coordinate value)
        center_x = width/2 + self.calib_config['center_x_offset']
        # print(x)
        if abs(x - center_x) < 10:
            yaw_output = 0
        elif abs(x - center_x) < width/6:
            yaw_output = math.copysign(1, x - center_x) + misc.val_map(x - center_x, -width/6, width/6, self.yaw_range[0] + 1, self.yaw_range[1] - 1)
        else:
            yaw_output = math.copysign(self.yaw_range[1], x - center_x)
        
        # print(x, center_x, yaw_output)
        # 转弯时前进步幅减小(When turning, decreases forward step length)
        #print(yaw_output)
        if abs(yaw_output) > 6: 
            x_output = 0.008
        else:
            x_output = self.x_max
        if abs(yaw_output) < 4:
            self.gait_manager.set_step(self.go_dsp, x_output, 0, int(-yaw_output), self.go_gait_param, arm_swap=self.go_arm_swap, step_num=0)
        else:
            self.gait_manager.set_step(self.turn_dsp, x_output, 0, int(-yaw_output), self.turn_gait_param, arm_swap=self.turn_arm_swap, step_num=0)
