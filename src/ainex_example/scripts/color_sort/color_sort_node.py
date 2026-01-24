#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/10
# @author:aiden
# 颜色分拣(color sorting)
import time
import copy
import rospy
import signal
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class ColorSortNode(Common):
    # y_min, y_max, x_min, x_max分别表示占图像的比例(y_min, y_max, x_min, and x_max represent respectively the proportions of the image)
    # 定义颜色块区域(define color block region)
    block_roi = [5/8, 7/8, 3/8, 5/8]
    # 定义放块动作名字(define name for the action of putting down block)
    right_hand_put_block_action_name = 'right_hand_put_block'   # 右手放块动作(put down block with right hand)
    left_hand_put_block_action_name = 'left_hand_put_block'     # 左手放块动作(put down block with left hand)

    # 图像处理时缩放到这个分辨率(When process the image, scale it to this resolution)
    image_process_size = [160, 120]
   
    def __init__(self, name):
        # 定义放块动作名字(define name for the action of putting down block)
        rospy.init_node(name)
        self.name = name
        self.count = 0          # 计数器(counter)
        self.running = True
        self.objects_info = []  # 存储识别结果(store recognition result)
        self.current_state = 'color_sort'   # 定义状态机(define state machine)

        # [['red', 'green', 'blue'], self.block_roi, self.image_process_size, self.set_blocks_color]
        # 颜色识别的参数设置,包括要识别的颜色、ROI区域、图像处理分辨率、生成颜色识别参数的方法。(set the parameters for color recognition, including the colors to be recognized, the ROI area, the image processing resolution, and the method for generating color recognition parameters)
        self.state = {'color_sort': [[500, 330], [['red', 'green', 'blue'], self.block_roi, self.image_process_size, self.set_blocks_color], False]}
        
        # 获取状态机初始头部姿态(obtain initial head posture of state machine)
        self.head_pan_init = self.state[self.current_state][0][0]   # 左右舵机的初始值(initial values of left and right servos)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial values of up and down servos)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        self.motion_manager.run_action('stand')
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
        
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            self.enter_func(None)
            self.start_srv_callback(None)  # 开启颜色识别(start color recognition)
            common.loginfo('start color_sort')
    # 关闭节点回调函数(exit node callback function)
    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)
    # 生成颜色块识别参数(generate color block recognition parameter)
    def set_blocks_color(self, colors, roi, image_process_size):
        # 根据颜色生成参数(according to color generation parameter)
        red_block_param = ColorDetect()
        red_block_param.color_name = colors[0]
        red_block_param.use_name = True
        red_block_param.detect_type = 'circle'
        red_block_param.roi.y_min = int(roi[0] * image_process_size[1])
        red_block_param.roi.y_max = int(roi[1] * image_process_size[1])
        red_block_param.roi.x_min = int(roi[2] * image_process_size[0])
        red_block_param.roi.x_max = int(roi[3] * image_process_size[0])
        red_block_param.image_process_size = image_process_size
        red_block_param.min_area = 10
        red_block_param.max_area = image_process_size[0]*image_process_size[1]

        green_block_param = copy.deepcopy(red_block_param)
        green_block_param.color_name = colors[1]
        
        blue_block_param = copy.deepcopy(red_block_param)
        blue_block_param.color_name = colors[2]

        return [red_block_param, green_block_param, blue_block_param]
    # 设置颜色服务回调函数(set color service callback function)
    def set_color_srv_callback(self, msg):
        # 设置颜色(set color)
        blocks_param = self.set_blocks_color(msg.data)
        self.detect_pub.publish([stairs_param])
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']
    
    # 获取颜色识别结果回调函数(obtain color recognition result callback function)
    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data

    # 颜色块处理函数(color block processing function)
    def color_sort_process(self, blocks_data):
        # 根据颜色播放动作(play action based on the color)
        if blocks_data is not None:
            if blocks_data.label == 'red':      # 处理红色色块(process color block)
                self.count += 1
                if self.count > 10:
                    self.count = 0
                    common.loginfo('red')
                    # 执行放块动作组(execute action group of putting down block)
                    self.motion_manager.run_action(self.right_hand_put_block_action_name)
            elif blocks_data.label == 'green':  # 处理绿色色块(process green block)
                self.count += 1
                if self.count > 10:
                    self.count = 0
                    common.loginfo('green')
                    # 执行放块动作(execute the action of putting down block )
                    self.motion_manager.set_servos_position(200, [[23, 400]])
                    time.sleep(0.2)
                    self.motion_manager.set_servos_position(300, [[23, 600]])
                    time.sleep(0.3)
                    self.motion_manager.set_servos_position(300, [[23, 400]])
                    time.sleep(0.3)
                    self.motion_manager.set_servos_position(200, [[23, 500]])
                    time.sleep(1.5)
            elif blocks_data.label == 'blue':     # 处理蓝色色块(process blue block)
                self.count += 1
                if self.count > 10:
                    self.count = 0
                    common.loginfo('blue')
                    self.motion_manager.run_action(self.left_hand_put_block_action_name)
        else:
            self.count = 0       # 识别丢失时计数清零(If the recognition is lost, reset the count to zero)

    def run(self):
        while self.running:
            if self.start:      # 根据状态机执行(execute based on the state machine)
                if self.state[self.current_state][2] == False:
                    self.state[self.current_state][2] = True
                    # 设置头部姿态(set head posture)
                    self.init_action(self.state[self.current_state][0][0], self.state[self.current_state][0][1])  # 头部姿态(head posture)
                    param = self.state[self.current_state][1][3](self.state[self.current_state][1][0],
                                                                 self.state[self.current_state][1][1],
                                                                 self.state[self.current_state][1][2])
                    self.detect_pub.publish(param)  # 颜色检测设置(set color detection)

                # 获取识别结果(obtain recognition result)
                blocks_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'circle':
                        blocks_data = object_info
                # 调用处理函数(call processing function)
                if self.current_state == 'color_sort':
                    self.color_sort_process(blocks_data)
                
                time.sleep(0.01)
            else:
                time.sleep(0.01)
        # 当前状态完成后关闭节点(close node after the current state is completed)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorSortNode('color_sort').run()
