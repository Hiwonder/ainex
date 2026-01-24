#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/03
# @author:aiden
# 十字路口检测(crossroad detection)
import time
import rospy
import signal
from ainex_sdk import common
from ainex_example.color_common import Common
# 导入巡线十字检测需要的模块
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class CrossDetectNode(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出(check three ROIs in order, and immediately exit if a black line is detected)
    # y_min, y_max, x_min, x_max分别表示占图像的比例, 即实际大小为y_min*height(y_min, y_max, x_min, and x_max represent respectively the proportion of the image occupied. The actual size is y_min*height)
    # 3个线条检测区，从上到下(three line detection region from top to bottom)
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]
    # 1个十字检测区域(one cross detection region)
    cross_roi = [1 / 4, 3 / 4, 0, 1]
    # 进入十字检测的图像阈值，视情况调整(Enter the image threshold for cross detection, and adjust based on the actual situation)
    enter_cross_detect_y = 240 / 480
    # 下采样后图像大小，可调整检测精度(Size of the image after down-sampling, and the detection accuracy can be adjusted)
    image_process_size = [160, 120]
    # 抬手动作名称(name of the action for raising up the hand)
    raise_right_hand_action_name = 'raise_right_hand'

    def __init__(self, name):
        # 初始化ROS节点(initialize ROS node)
        rospy.init_node(name)
        self.name = name
        # 是否继续运行的标志(flag used to indicate whether the program should continue running)
        self.running = True
        self.count = 0
        # 存储检测到的对象信息(store information about detected objects)
        self.objects_info = []
        # 定义两个状态  
        self.current_state = 'visual_patrol'  #巡线状态(line following state)
        self.next_state = 'cross_detect'      #十字路口状态(crossing state)
        # 状态对应的参数:头部姿态,颜色检测参数,是否初始化(the parameters for each state: head posture, color detection parameter, initialization flag)
        # 通过字典实现不同状态下的配置：visual_patrol巡线状态；cross_detect十字路口状态(achieving configurations under different states through a dictionary: "visual_patrol" line following state; "cross_detect" crossroad state)
        self.state = {'visual_patrol':  [[500, 260], ['black', self.line_roi, self.image_process_size, self.set_visual_patrol_color], False],
                      'cross_detect':   [[500, 260], ['black', self.cross_roi, self.image_process_size, self.set_cross_color], False]}
        
        # 获取状态对应的头部初始姿态(obtain state corresponding to head initial posture)
        self.head_pan_init = self.state[self.current_state][0][0]   # 左右舵机的初始值(initial value of the left-right servo)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial value of the up-down servo)
        # 初始化父类,设置头部角度(initialize the parent class and set the initial angle of the head)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        # 创建巡线控制实例(create line following control instance)
        self.visual_patrol = VisualPatrol(self.gait_manager)

        # 减小巡线步幅，提高其他标志检测稳定性(Decrease the stride of line following, improving the stability of other flag detection)
        self.visual_patrol.update_go_gait(dsp=[300, 0.2, 0.02], x_max=0.01)
        self.visual_patrol.update_turn_gait(dsp=[400, 0.2, 0.02], x_max=0.01) 

        # Ctrl+C 退出(Ctrl+C exit)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        # 创建发布颜色识别参数的publisher
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)

        self.motion_manager.run_action('walk_ready')  # 准备站立(ready to stand up)
        # 是否自动开始(whether to automatically start)
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            self.enter_func(None)
            #self.set_color_srv_callback('black', self.cross_roi)
            self.start_srv_callback(None)  # 开启颜色识别(start color recognition)
            common.loginfo('start cross detect')
    
    # Ctrl+C 退出回调(Ctrl+C exit callback)
    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)
     # 根据巡线参数生成颜色识别配置(generate color recognition configuration based on line following parameter)
    def set_visual_patrol_color(self, color, roi, image_process_size):
        # 设置巡线颜色(set line following color)
        line_param = ColorDetect()
        line_param.color_name = color
        line_param.use_name = True
        line_param.detect_type = 'line'
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
    
    # 生成十字检测颜色检测参数(generate cross and color detection parameters)
    def set_cross_color(self, color, roi, image_process_size):
        cross_param = ColorDetect()
        cross_param.color_name = color
        cross_param.use_name = True
        cross_param.detect_type = 'cross'
        cross_param.image_process_size = image_process_size
        cross_param.roi.y_min = int(roi[0] * image_process_size[1])
        cross_param.roi.y_max = int(roi[1] * image_process_size[1])
        cross_param.roi.x_min = int(roi[2] * image_process_size[0])
        cross_param.roi.x_max = int(roi[3] * image_process_size[0])
        cross_param.min_area = 10
        cross_param.max_area = image_process_size[0]*image_process_size[1]
        
        return cross_param


    # 状态初始化(initialize state)
    def state_init(self, current_state, next_state):
        # 不同阶段的初始化(initialization in different phrase)
        if self.state[current_state][2] == False:
            # 设置头部姿态(set head posture)
            self.state[current_state][2] = True
            self.init_action(self.state[current_state][0][0], self.state[current_state][0][1])  # 头部姿态(head posture)
            # 生成检测参数并发布(generate detection parameter and publish)
            param1 = self.state[current_state][1][3](self.state[current_state][1][0], self.state[current_state][1][1], self.state[current_state][1][2])
            param2 = self.state[next_state][1][3](self.state[next_state][1][0], self.state[next_state][1][1], self.state[next_state][1][2])
            self.detect_pub.publish([param1, param2])  

    # 进入十字检测的条件(the condition for entering cross detection)
    def enter_cross_detect(self, cross_data):
        # 根据十字区域检测结果判断(determine based on the cross region detection result)
        if cross_data is not None:
            
            if max(cross_data.y, cross_data.left_point[1], cross_data.right_point[1]) > self.enter_cross_detect_y * cross_data.height:
                self.count += 1
                print(self.count)
                if self.count > 2:
                    self.count = 0
                    self.gait_manager.stop()
                    common.loginfo('cross detect')
                    return True
            else:
                self.count = 0
        else:
            self.count = 0
        return False

    # 退出十字检测后的动作(the action after exiting the cross detection)
    def exit_cross_detect(self):
        # 播放动作(play action)
        # 更新步态(update gait)
        # 进入巡线状态(enter line following state)
        self.gait_manager.disable()
        self.motion_manager.run_action(self.raise_right_hand_action_name)
        walking_param = self.gait_manager.get_gait_param()
        walking_param['body_height'] = 0.015
        walking_param['pelvis_offset'] = 7
        walking_param['step_height'] = 0.02
        walking_param['hip_pitch_offset'] = 10
        walking_param['z_swap_amplitude'] = 0.006
        self.gait_manager.set_step([600, 0.2, 0.04], 0.02, 0, 0, walking_param, 0, 4)
        self.motion_manager.run_action('stand')
        return True

    # 颜色识别回调(color recognition callback)
    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data

    def run(self):
        while self.running:
            if self.start:
                line_data = None
                cross_data = None
                for object_info in self.objects_info:
                    print(object_info.type)
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'cross':
                        cross_data = object_info
                
                if self.current_state == 'visual_patrol':
                    if line_data is not None:
                        self.visual_patrol.process(line_data.x, line_data.width)
                elif self.current_state == 'cross_detect':
                    if self.exit_cross_detect():
                        self.current_state = 'visual_patrol'
                        self.next_state = 'cross_detect'
                        self.state[self.current_state][2] = False
                
                if self.next_state == 'cross_detect':
                    if self.enter_cross_detect(cross_data):
                        self.current_state = 'cross_detect'
                        self.next_state = 'visual_patrol'
                
                self.state_init(self.current_state, self.next_state)

                time.sleep(0.01)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    CrossDetectNode('cross_detect').run()
