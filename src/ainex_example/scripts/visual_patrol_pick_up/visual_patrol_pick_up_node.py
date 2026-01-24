#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 巡线抓取(line following and grasping)
import time
import rospy
import signal
from ainex_sdk import misc, common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ROI

class VisualPatrolPickUp(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出(check three ROIs in order, and immediately exit if a black line is detected)
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]

    block_roi = [0, 2 / 3, 0, 1]
    intersection_roi = [100 / 480, 340 / 480, 0, 1]
    # 向左左行走状态下的目标位置阈值(Threshold values for target position in walking left phase)
    enter_crawl_left_y = 290 / 480 # 当检测到的标识像素坐标y值占图像的比例大于此值时进入此阶段(When the detected object's y-coordinate in the image is greater than this value, it enters this phrase)
    crawl_left_x_stop = 265 / 640  # 左行走的x轴目标位置范围可在ApproachObject里设置)时停止前后移动(The target range of the x-axis position for walking left (can be set in ApproachObject); when the robot reaches this range, it stops moving forward or backward)
    crawl_left_y_stop = 290 / 480  # 左行走的y轴目标位置(范围可在ApproachObject里设置)时停止横向移动(The target range of the y-axis position for walking left (can be set in ApproachObject); when the robot reaches this range, it stops moving horizontally)
    crawl_left_yaw_stop = 0        # 当检测到的标识角度在此值附近(范围可在ApproachObject里设置)时停止旋转移动(The target range of the detected object's angle (can be set in ApproachObject); when the detected object's angle is within this range, the robot stops rotating)
    # 向左左行走状态下的目标位置阈值，原理同上(Threshold values for target position in walking right phase)
    enter_crawl_right_y = 280 / 480
    crawl_right_x_stop = 390 / 640
    crawl_right_y_stop = 272 / 480
    crawl_right_yaw_stop = 0
    # 放块状态下的目标位置阈值(Threshold values for target position in block placing phase)
    enter_place_block_y = 320 / 480
    place_block_x_stop = 320 / 640
    place_block_y_stop = 350 / 480
    place_block_yaw_stop = 0

    crawl_left_action_name = 'crawl_left'   # 左行走动作名称(name of the walking left action)
    crawl_right_action_name = 'crawl_right' # 右行走动作名称(name of the walking right action)
    place_block_action_name = 'place_block' # 放置方块动作名称(name of the block placing action)

    image_process_size = [160, 120]

    def __init__(self, name):
        rospy.init_node(name)       # 初始化ROS节点(initialize ROS node)
        self.name = name
        self.running = True         # 运行标志(run flag)
        self.count = 0              # 计数器(counter)
        self.objects_info = []      # 存储识别结果(store recognition result)
        self.current_state = "visual_patrol"    # 当前状态(current state)
        self.next_state = "crawl_left"          # 下一状态(next state)

        # 定义状态机(define state machine)
        self.state = {'visual_patrol':  [[500, 260], ['black', self.line_roi, self.image_process_size, self.set_visual_patrol_color], False],           # 巡线状态(line following state)
                      'crawl_left':   [[500, 260], ['green', self.block_roi, self.image_process_size, self.set_block_color], False],                    # 左爬行状态(left crawling state)
                      'crawl_right': [[500, 260], ['green', self.block_roi, self.image_process_size, self.set_block_color], False],                     # 右爬行状态(right crawling state)
                      'place_block':   [[500, 260], ['black', self.intersection_roi, self.image_process_size, self.set_intersection_color], False]}     # 放块状态(block placing state)
        
        self.head_pan_init = self.state[self.current_state][0][0]   # 左右舵机的初始值(initial value of the left-right servo)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial value of the up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init) # 初始化父类(initialize parent class)

        self.visual_patrol = VisualPatrol(self.gait_manager)        # 巡线节点(line following node)
        self.visual_patrol.update_go_gait(x_max=0.01)
        self.visual_patrol.update_turn_gait(x_max=0.01)
        self.approach_object = ApproachObject(self.gait_manager)    # 逼近节点(approaching node)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
        self.motion_manager.run_action('walk_ready')

        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            self.enter_func(None)
            self.start_srv_callback(None)  # 开启颜色识别(start color recognition)
            common.loginfo('start crawl')

    # 关闭节点(close node)
    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    # 生成巡线颜色识别参数(generate line following color recognition parameter)
    def set_visual_patrol_color(self, color, roi, image_process_size):
        # 设置巡线颜色(set line following color)
        line_param = ColorDetect()
        line_param.color_name = color
        line_param.use_name = True
        line_param.detect_type = 'line'     # 检测类型为线条(detection type is line)
        line_param.image_process_size = image_process_size
        line_param.line_roi.up.y_min = int(roi[0][0] * image_process_size[1])
        line_param.line_roi.up.y_max = int(roi[0][1] * image_process_size[1])
        line_param.line_roi.up.x_min = int(roi[0][2] * image_process_size[0])
        line_param.line_roi.up.x_max = int(roi[0][3] * image_process_size[0])

        line_param.line_roi.center.y_min = int(roi[1][0] * image_process_size[1])
        line_param.line_roi.center.y_max = int(roi[1][1] * image_process_size[1])
        line_param.line_roi.center.x_min = int(roi[1][2] * image_process_size[0])
        line_param.line_roi.center.x_max = int(roi[1][3] * image_process_size[0])

        line_param.line_roi.down.y_min = int(roi[2][0] * image_process_size[1])
        line_param.line_roi.down.y_max = int(roi[2][1] * image_process_size[1])
        line_param.line_roi.down.x_min = int(roi[2][2] * image_process_size[0])
        line_param.line_roi.down.x_max = int(roi[2][3] * image_process_size[0])

        line_param.min_area = 1
        line_param.max_area = image_process_size[0] * image_process_size[1]
        
        return line_param
    # 生成方块颜色识别参数(generate block color recognition parameter)
    def set_block_color(self, color, roi, image_process_size):
        block_param = ColorDetect()
        block_param.color_name = color
        block_param.detect_type = 'circle'
        block_param.use_name = True
        block_param.image_process_size = self.image_process_size
        block_param.roi.y_min = int(roi[0] * image_process_size[1])
        block_param.roi.y_max = int(roi[1] * image_process_size[1])
        block_param.roi.x_min = int(roi[2] * image_process_size[0])
        block_param.roi.x_max = int(roi[3] * image_process_size[0])
        block_param.min_area = 10
        block_param.max_area = image_process_size[0]*image_process_size[1]
        
        return block_param
    # 生成交叉点颜色识别参数(generate intersection color recognition parameter)
    def set_intersection_color(self, color, roi, image_process_size):
        intersection_param = ColorDetect()
        intersection_param.color_name = color
        intersection_param.detect_type = 'intersection'
        intersection_param.use_name = True
        intersection_param.image_process_size = image_process_size
        intersection_param.roi.y_min = int(roi[0] * image_process_size[1])
        intersection_param.roi.y_max = int(roi[1] * image_process_size[1])
        intersection_param.roi.x_min = int(roi[2] * image_process_size[0])
        intersection_param.roi.x_max = int(roi[3] * image_process_size[0])
        intersection_param.min_area = 10
        intersection_param.max_area = image_process_size[0] * image_process_size[1]

        return intersection_param
    # 设置颜色服务回调函数(set color service callback function)
    def set_color_srv_callback(self, msg):
        # 设置颜色(set color)
        block_param = self.set_block_color(msg.data)
        line_param = self.set_visual_patrol_color('black')
        intersection_param = self.set_intersection_color('black')
        
        self.detect_pub.publish([line_param, block_param, intersection_param])
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']
    # 获取颜色识别结果回调函数(obtain color recognition result callback function)
    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data
    # 状态初始化函数(state initialization function)
    def state_init(self, current_state, next_state):
        # 不同阶段的初始化(initialization in different phrase)
        if self.state[current_state][2] == False:
            self.state[current_state][2] = True
            self.init_action(self.state[current_state][0][0], self.state[current_state][0][1])  # 头部姿态(head posture)
            param1 = self.state[current_state][1][3](self.state[current_state][1][0], self.state[current_state][1][1], self.state[current_state][1][2])
            param2 = self.state[next_state][1][3](self.state[next_state][1][0], self.state[next_state][1][1], self.state[next_state][1][2])
            self.detect_pub.publish([param1, param2])  # 颜色检测设置(color detection settings)
            common.loginfo('%s init' % current_state)
    # 进入左行走判断函数(enter left walking determination function)
    def enter_crawl_left(self, block_data):
        if block_data is not None:
            # 如果色块的y坐标大于高度的1/4,说明有一部分进入ROI区域(If the y-coordinate of the block is greater than 1/4 of its height, part of it is in the ROI area)
            if block_data.y > block_data.height / 4:
                self.count += 1
                if self.count > 5:  # 连续检测到满足条件,切换到左行走状态(If the condition is satisfied continuously for more than 5 times, switch to the walking left phase)
                    self.count = 0
                    self.gait_manager.disable()
                    self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(Move the hand back to prevent obstruction)
                    return True
            else:
                self.count = 0
        else:
            self.count = 0
        return False
    # 进入右行走判断函数(Enter the walking right judgment function)
    def enter_crawl_right(self, block_data):
        if block_data is not None:
            if block_data.y > block_data.height / 4:
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    self.gait_manager.disable()
                    self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(Move the hand back to prevent obstruction)
                    return True
            else:
                self.count = 0
        else:
            self.count = 0
        return False
    # 进入右爬行判断函数(Enter the crawling right judgment function)
    def enter_place_block(self, line_data):
        if line_data is not None:
            if max(line_data.y, line_data.left_point[1], line_data.right_point[1]) > self.enter_place_block_y * line_data.height:
                self.count += 1
                if self.count > 2:
                    self.count = 0
                    self.gait_manager.stop()
                    common.loginfo('intersection detect')
                    return True
            else:
                self.count = 0
        else:
            self.count = 0
        return False
    # 退出左行走判断函数(Exit the walking left judgment function)
    def exit_crawl_left(self, block_data):
        if block_data is not None:
            if self.approach_object.process(block_data.y, block_data.x, block_data.angle, 
                                            self.crawl_left_y_stop*block_data.height, self.crawl_left_x_stop*block_data.width, self.crawl_left_yaw_stop, block_data.width, block_data.height):
                self.gait_manager.disable()
                common.loginfo('crawl_left')
                self.motion_manager.run_action(self.crawl_left_action_name)
                self.visual_patrol.update_go_gait(arm_swap=0)
                self.visual_patrol.update_turn_gait(arm_swap=0)
                return True
        return False
    # 退出右行走判断函数(Exit the walking right judgment function)
    def exit_crawl_right(self, block_data):
        if block_data is not None:
            if self.approach_object.process(block_data.y, block_data.x, block_data.angle, 
                                            self.crawl_right_y_stop*block_data.height, self.crawl_right_x_stop*block_data.width, self.crawl_right_yaw_stop, block_data.width, block_data.height):
                self.gait_manager.disable()
                common.loginfo('crawl_right')
                self.motion_manager.run_action(self.crawl_right_action_name)
                self.visual_patrol.update_go_gait(arm_swap=0)
                self.visual_patrol.update_turn_gait(arm_swap=0)
                return True
        return False
    # 退出放块判断函数(Exit the block placing judgment function)
    def exit_place_block(self, line_data):
        if line_data is not None:
            if self.approach_object.process(max(line_data.y, line_data.left_point[1], line_data.right_point[1]), line_data.x, line_data.angle, 
                                            self.place_block_y_stop*line_data.height, self.place_block_x_stop*line_data.width, self.place_block_yaw_stop, line_data.width, line_data.height):
                self.gait_manager.disable()  # 关闭步态控制(close gait control)
                walking_param = self.gait_manager.get_gait_param()  # 获取当前的步态参数(get the current gait parameters)
                walking_param['body_height'] = 0.015                # 设置身体高度,单位米(set the body height in meters)
                walking_param['pelvis_offset'] = 7                  # 设置骨盆位置的前后偏移量,单位度(set the front and rear offset of the pelvis position in degrees)
                walking_param['step_height'] = 0.02                 # 设置步高,单位米(set the step height in meters)
                walking_param['hip_pitch_offset'] = 20              # 设置髋关节角度偏移量,单位度(set the hip joint angle offset in degrees)
                walking_param['z_swap_amplitude'] = 0.006           # 设置左右足高度交替时的振幅,单位米(set the amplitude of the left and right foot height alternation in meters)
                # 调用set_step 设置步态参数,包括步长、步频等(Call set_step to set the gait parameters, including step length, step frequency, etc)
                self.gait_manager.set_step([500, 0.2, 0.035], 0.02, 0, 0, walking_param, 0, 3)           
                common.loginfo('place_block')
                self.motion_manager.run_action(self.place_block_action_name)
                return True
        return False
    # 程序主循环(program main loop)
    def run(self):
        while self.running:
            if self.start:  # 如果start为True,表示程序开始运行(If the start is Ture, it indicates the program starts to run)
                line_data = None
                block_data = None
                intersection_data = None   
                # 从识别结果中提取出线、方块、交叉点的数据(obtain data of line, block, and cross points from the recognition result)
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'circle':
                        block_data = object_info
                    if object_info.type == 'intersection':
                        intersection_data = object_info

                # 当前阶段处理完成，回到巡线(The current processing is completed, and return to line following)
                if self.current_state == 'visual_patrol':
                     # 如果当前状态是巡线(if the current state is line following)
                    if line_data is not None:
                        # 如果检测到线条(if the line is detected)
                        # 执行巡线过程(perform line following)
                        self.visual_patrol.process(line_data.x, line_data.width)
                elif self.current_state == 'crawl_left':
                    # 如果当前状态是左行走(if the current state is right walking)
                    if self.exit_crawl_left(block_data):
                        # 如果完成左行走(if finishing walking to the left)
                        self.current_state = 'visual_patrol'
                        self.next_state = 'crawl_right'
                        self.state[self.current_state][2] = False
                    else:
                        time.sleep(0.8)  # 等机体平稳下来(wait for the robot to stabilize)
                elif self.current_state == 'crawl_right':
                    # 右行走状态(the state of walking right)
                    if self.exit_crawl_right(block_data):
                        # 如果完成右行走(if finishing walking to the right)
                        self.current_state = 'visual_patrol'
                        self.next_state = 'place_block'
                        self.state[self.current_state][2] = False
                    else:
                        time.sleep(0.8)
                elif self.current_state == 'place_block':
                    # 放块状态(the state of putting down the block)
                    if self.exit_place_block(intersection_data):
                        self.running = False
                    else:
                        time.sleep(0.8)

                # # 状态切换判断，是否退出巡线，进入下一阶段(Determine state switching; whether to exit the line following and enter next phrase)
                if self.next_state == 'crawl_left':
                    if self.enter_crawl_left(block_data):
                        self.current_state = 'crawl_left'
                        self.next_state = 'visual_patrol'
                elif self.next_state == 'crawl_right':
                    if self.enter_crawl_right(block_data):
                        self.current_state = 'crawl_right'
                        self.next_state = 'visual_patrol'
                elif self.next_state == 'place_block':
                    if self.enter_place_block(intersection_data):
                        self.current_state = 'place_block'
                        self.next_state = 'visual_patrol'

                self.state_init(self.current_state, self.next_state)
                
                time.sleep(0.01)  # 防止空载(prevent the robot from running under no loading)
            else:
                time.sleep(0.01)
        # 程序结束,关闭节点(The program is over, close node)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrolPickUp('visual_patrol_pick_up').run()
