#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/11
# @author:aiden
# 接近物体逻辑处理(the logic processing of approaching an object)
import math
from ainex_sdk import misc, common

class ApproachObject:
    def __init__(self, gait_manager, step_mode=1, debug=False):
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        self.debug = debug
        self.count_stop = 0
        self.stop_count = 0
        self.x_range = [-0.01, 0.01]  # 前进脚步大小单位m(step size for forward motion, in units of m)
        self.y_range = [-0.01, 0.01]  # 前进脚步大小单位m(step size for forward motion, in units of m)
        self.yaw_range = [-8, 8]  # 转弯范围正负10度(the range of turning angle is -10 to 10, in units of degrees)
        self.gait_manager = gait_manager
        
        self.gait_param = self.gait_manager.get_gait_param()
        self.gait_param['pelvis_offset'] = 5
        self.gait_param['step_height'] = 0.02
        self.gait_param['z_swap_amplitude'] = 0.006
        self.dsp = [400, 0.2, 0.02]
        self.arm_swap = 0

        self.x_approach_value = 50 
        self.y_approach_value = 40
        self.yaw_approach_value = 7

        self.step_mode = step_mode
        
    def update_stop_count(self, num):
        self.stop_count = num

    def update_gait(self, dsp=None, arm_swap=None, walking_param=None, step_mode=None):
        if walking_param is not None:
            self.gait_param = walking_param
        if step_mode is not None:
            self.step_mode = step_mode
        if dsp is not None:
            self.dsp = dsp
        if arm_swap is not None:
            self.arm_swap = arm_swap

    def update_gait_range(self, x_range=None, y_range=None, yaw_range=None):
        if x_range is not None:
            self.x_range = x_range
        if y_range is not None:
            self.y_range = y_range
        if yaw_range is not None:
            self.yaw_range = yaw_range

    def update_approach_stop_value(self, x_approach_value=None, y_approach_value=None, yaw_approach_value=None):
        if x_approach_value is not None:
            self.x_approach_value = x_approach_value
        if y_approach_value is not None:
            self.y_approach_value = y_approach_value
        if yaw_approach_value is not None:
            self.yaw_approach_value = yaw_approach_value

    def process(self, max_y, center_x, angle, x_stop, y_stop, yaw_stop, width, height):
        y_stop = y_stop + self.calib_config['center_x_offset'] 
        # 根据线的倾斜角度进行旋转调整(rotate and adjust based on the slope angle of the line)
        if abs(angle - yaw_stop) < 10:
            yaw_output = misc.val_map(angle - yaw_stop, -8, 8, self.yaw_range[0], self.yaw_range[1])
        else:
            yaw_output = math.copysign(self.yaw_range[1], angle)
        if abs(yaw_output) < 3:
            yaw_output = math.copysign(3, angle)
        if abs(angle) <= self.yaw_approach_value:
            yaw_output = 0

        # print(angle, yaw_stop, yaw_output)
        if abs(y_stop - center_x) < width / 8:
            y_output = misc.val_map(y_stop - center_x, -width / 8, width / 8, self.y_range[0], self.y_range[1])
        else:
            y_output = math.copysign(self.y_range[1], y_stop - center_x)
        if abs(y_output) < 0.005:
            y_output = math.copysign(0.005, y_stop - center_x)
        if abs(y_stop - center_x) <= self.y_approach_value:
            y_output = 0

        if abs(max_y - x_stop) > height / 4:
            x_output = math.copysign(self.x_range[1], x_stop - max_y)
        elif abs(max_y - x_stop) <  height / 8:
            x_output = math.copysign(0.005, x_stop - max_y)
        else:
            x_output = misc.val_map(x_stop - max_y, -height / 4, height / 4, self.x_range[0], self.x_range[1])
        if abs(x_output) < 0.005:
            x_output = math.copysign(0.005, x_stop - max_y)
        if abs(x_stop - max_y) <= self.x_approach_value:
            x_output = 0
        
        if abs(yaw_output) >= 5:
            if x_output >= 0.01:
                x_output = 0.01
            else:
                x_output = 0
        
        # print(x_output, y_output, yaw_output, center_x, max_y, angle)
        # yaw_output, x_output = 0, 0
        # print(abs(angle), abs(max_y - x_stop), abs(y_stop - center_x)) 
        if self.debug: 
            print(center_x, max_y, angle)
        else:
            if x_output != 0 or y_output != 0 or yaw_output != 0:
                self.count_stop = 0
                self.gait_manager.set_step(self.dsp, round(x_output, 4), round(y_output, 4), -int(yaw_output), self.gait_param, arm_swap=self.arm_swap, step_num=self.step_mode)
            else:
                self.count_stop += 1
                self.gait_manager.stop()  # 需要先关闭行走(disable the walking function)
        if self.count_stop > self.stop_count:
            self.count_stop = 0
            print('approach finish')
            return True
        else:
            return False
