#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/20
# @author:aiden
# 自主搬运(autonomous transport)
import time
import math
import copy
import rospy
import signal
from std_srvs.srv import Empty, EmptyResponse   # 导入ROS Empty服务定义
from std_msgs.msg import Float64, String
from ainex_sdk import pid, misc, common
from ainex_example.color_common import Common
from ainex_example.pid_track import PIDTrack
from apriltag_ros.msg import AprilTagDetectionArray # 导入ROS Empty服务定义
from ainex_example.approach_object import ApproachObject    # 导入ApproachObject类
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect  # 导入自定义消息类型

class AutonomousTransportNode(Common):
    block_x_stop = 320 / 640  # 当检测到的标识像素坐标x值占图像的比例在此值附近时停止前后移动(范围可在ApproachObject里设置)时停止前后移动(When the detected object's x-coordinate in the image is within a certain range (specified in the ApproachObject function) based on the proportion of the image, the robot will stop moving forward or backward)
    block_y_stop = 220 / 480  # 当检测到的标识像素坐标y值占图像的比例在此值附近时停止横向移动(范围可在ApproachObject里设置)时停止横向移动(When the detected object's y-coordinate in the image is within a certain range (specified in the ApproachObject function) based on the proportion of the image, the robot will stop moving left or right)
    block_yaw_stop = 0        # 当检测到的标识角度在此值附近时停止旋转移动(范围可在ApproachObject里设置)时停止旋转移动(When the detected object's orientation angle is within a certain range (specified in the ApproachObject function), the robot will stop rotating)

    tag_y_stop = 240 / 480    # AprilTag在图像中的停止位置比例(AprilTag's stop position proportion in the image)

    move_up_action_name = 'move_up'     # 定义的抬起动作的名称(the name used to define the action of lifting)
    put_down_action_name = 'put_down'   # 定义的放下动作的名称(the name used to define the action of putting down)

    image_process_size = [160, 120]     # 图像处理的分辨率大小(the resolution size of image processing)
    
    # 搬运颜色对应的tag(transport the tag with corresponding color)
    transport_dict = {'red': 1, 'green': 2, 'blue': 3}
    
    # 搬运顺序(the order of transport)
    transport_index = ['red', 'green', 'blue']
    
    def __init__(self, name):
        rospy.init_node(name)       # ROS节点初始化(initialize ROS node)
        self.name = name            # 节点名称(the name of node)
        self.running = True         # 运行状态(running state)
        self.count_miss = 0         # 丢失检测次数(the number of times for detecting losing)
        self.object_info = None     # 颜色识别结果(color recognition result)
        self.start_index = 0        # 搬运顺序索引(transport order index)
        self.y_stop = self.block_y_stop # tag停止像素坐标y值占比(tag stop pixel coordinate y-value ratio)
        self.tag_data = {'1': False, '2': False, '3': False}            # 检测到的tag数据(the detected tag data)
        self.remain_block_list = copy.deepcopy(self.transport_index)    # 剩余搬运块顺序(transport order of the remaining blocks)
        self.current_color = self.transport_index[0]                    # 当前搬运颜色(current transport color)
        self.body_track_state = 'approach'      # 躯体追踪状态(body tracking status)
        self.head_track_state = 'track_block'   # 头部追踪状态(head tracking status)
        self.transport_state = 'pick_block'     # 搬运状态(transport status)
        self.start_find_block = False           # 开始找方块标志(flag indicating the start of finding block)
        # 初始化头部位置(initialize head position)
        self.head_pan_init = 500   # 左右舵机的初始值(initial values of left and right servos)
        self.head_tilt_init = 280  # 上下舵机的初始值(initial values of up and down servos)
        self.head_time_stamp = rospy.get_time() # 上一次头部运动时间(the time of the previous head motion)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)  # 初始化Common类(initialize Common class)
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        self.approach_object = ApproachObject(self.gait_manager)         # 初始化ApproachObject类(initialize ApproachObject class)
        
        # 更新ApproachObject的停止判断阈值(update the determination threshold for stopping ApproachObject)
        self.approach_object.update_stop_count(1)                       # 停止判断的平稳次数阈值(the stable count threshold for stop judgment)
        self.approach_object.update_approach_stop_value(10, 40, 7)      # 设置判断停止的误差阈值,分别对应x/y/yaw(set the error threshold for judging stop, corresponding to x/y/yaw)
        self.approach_object.update_gait_range(x_range=[-0.015, 0.015]) # 设置步态移动范围(set the gait movement range)

        # 获取步态参数(Obtain gait parameter)
        self.walking_param = self.gait_manager.get_gait_param()
        self.walking_param['step_height'] = 0.025    # 步高，单位：米(step height, in units of meters)
        self.walking_param['pelvis_offset'] = 3      # 盆骨偏移(pelvic offset)
        self.walking_param['hip_pitch_offset'] = 15  # 髋关节偏移(hip joint offset)
        self.walking_param['body_height'] = 0.035    # 身体高度(body height)
        # 更新ApproachObject的步态参数(update the gait parameters of the ApproachObject)
        self.approach_object.update_gait(walking_param=self.walking_param)  

        
        self.motion_manager.run_action('walk_ready') # 运行站立动作(perform standing up action)
        signal.signal(signal.SIGINT, self.shutdown) # 注册SIGINT处理函数(register SIGINT processing function)

        # 头部的pid追踪(PID tracking for head)
        self.head_pan_range = [125, 875]   # 左右转动限制在这个范围， 125为右(the range of left and right rotation is limited to this range, with 125 as the right)
        self.head_tilt_range = [250, 500]  # 上下限制在这个范围， 250为下(the range of up and down is limited to this range, with 250 as down)
        self.rl_dis = None
        self.ud_dis = None
        self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        self.pid_ud = pid.PID(0.1, 0.0, 0.001)
         # 初始化PID追踪类(initialize PID tracking class)
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)

        # 躯体的追踪参数(tracking parameter of the body)
        self.yaw_stop = 0  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值(the difference between the left and right head servos pulse width and the neutral position 500 when the body stops rotating)

        # 头部预设5个搜寻位置，左右舵机，上下舵机，时间ms(the head is preset with 5 search positions, left and right servos, up and down servos, and time in ms)
        # left_down, left_up, center_up, right_up, right_down
        self.head_move_position_block = [[650, 300, 1000],
                                         [650, 500, 1000],
                                         [500, 500, 1000],
                                         [350, 500, 1000],
                                         [350, 300, 1000]
                                         ]
        self.head_move_position_tag = [[550, 300, 1000],
                                       [550, 500, 1000],
                                       [500, 500, 1000],
                                       [450, 500, 1000],
                                       [450, 300, 1000]
                                       ]
        self.head_move_position = copy.deepcopy(self.head_move_position_block)
        
        rospy.ServiceProxy('/color_detection/start', Empty)()   # 启动颜色识别(enable color recognition)
        self.set_color_srv_callback(String(self.current_color)) # 设置识别蓝色(set to recognize blue)
        rospy.ServiceProxy('/apriltag_ros/stop', Empty)()       # 停止apriltag识别(stop apriltag recognition)

        # 订阅apriltag识别结果(subscribe to apriltag recognition result)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        # rospy.ServiceProxy('/color_detection/stop', Empty)()
        # 停止apriltag识别(stop apriltag recognition)
        # rospy.ServiceProxy('/apriltag_ros/stop', Empty)()
        # rospy.ServiceProxy('/apriltag_ros/start', Empty)()
        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        # 设置颜色服务回调函数(set color service callback function)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  

        # 是否启动(whether to enable)
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify to ready for color recognition, and only display the original image of the camera)
            self.enter_func(None)
            self.start_srv_callback(None)  # 开启颜色识别(enable color recognition)
            common.loginfo('start transport block')
    # 关闭节点(close node)
    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    # 设置颜色服务回调函数(set color service callback function)
    def set_color_srv_callback(self, msg):
        # 设置追踪颜色(set color to be tracked)
        param = ColorDetect()
        param.color_name = msg.data
        param.detect_type = 'rect'
        param.use_name = True
        param.image_process_size = self.image_process_size
        param.min_area = 10
        param.max_area = 16 * self.image_process_size[0] * self.image_process_size[1]
        self.detect_pub.publish([param])
        common.loginfo('{} set_color {}'.format(self.name, msg.data))
        return [True, 'set_color']
    
    # 颜色识别结果回调函数(color recognition result callback function)
    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        for object_info in msg.data:
            if object_info.type == 'rect':
                self.object_info = [object_info.x, object_info.y, object_info.angle, object_info.width, object_info.height]
                if self.transport_state == 'pick_block' and self.head_track_state != 'stop' and not self.start_find_block:
                    self.rl_dis, self.ud_dis = self.head_track_process(object_info.x, object_info.y, object_info.width, object_info.height)
                elif self.head_track_state == 'stop':
                    self.rl_dis, self.ud_dis = None, None

    # apriltag识别结果回调函数(apriltag recognition result callback function)
    def tag_callback(self, msg):
        if msg.detections != []:
            for i in msg.detections:
                if i.id[0] == self.transport_dict[self.current_color]:
                    roll, pitch, yaw = common.qua2rpy(i.pose.pose.pose.orientation)
                    x = int((i.corners[0].x + i.corners[2].x)/2)
                    y = int((i.corners[0].y + i.corners[2].y)/2)
                    self.object_info = [x, y, math.degrees(yaw), 640, 480]
                    if self.transport_state == 'place_block' and self.head_track_state != 'stop' and not self.start_find_block:
                        self.rl_dis, self.ud_dis = self.head_track_process(x, y, 640, 480)
                    elif self.head_track_state == 'stop':
                        self.rl_dis, self.ud_dis = None, None
                self.tag_data[str(i.id[0])] = True

    # 初始化搬运方块(initialize block transport)
    def find_block_init(self):
        self.y_stop = self.block_y_stop
        self.head_track_state = 'track_block'
        self.transport_state = 'pick_block'
        # 更新接近停止的判断阈值(update the judgment threshold for approaching stop)
        # 10:x方向位移误差阈值,单位是像素(10: x-direction displacement error threshold, in unit of pixels)
        # 40:y方向位移误差阈值,单位是像素(40: y-direction displacement error threshold, in unit of pixels)
        # 7:航向角误差阈值,单位是度(7: yaw angle error threshold, in unit of degrees)
        self.approach_object.update_approach_stop_value(10, 40, 7)
        self.walking_param['hip_pitch_offset'] = 15  # 髋关节角度偏移量(hip joint angle offset)

        # 更新步态参数(update gait parameter)
        # 步长,步高,步速(stride, step height, and walking speed)
        # 行走稳定步数(walking stable step number)
        # 步态参数(gait parameter)
        self.approach_object.update_gait([400, 0.2, 0.025], 30, self.walking_param)
        # 复制block的头部位置列表(copy the head position list of block)
        self.head_move_position = copy.deepcopy(self.head_move_position_block)
        # 从剩余列表中移除当前颜色(remove current color from remaining list)
        self.remain_block_list.remove(self.current_color)
        if self.remain_block_list == []:
            self.running = False
            # self.remain_block_list = copy.deepcopy(self.transport_index)
        else:
            self.current_color = self.remain_block_list[0]
        rospy.ServiceProxy('/color_detection/start', Empty)()
        self.set_color_srv_callback(String(self.current_color))  # 设置识别蓝色(set to recognize blue)
        rospy.ServiceProxy('/apriltag_ros/stop', Empty)()

    # 初始化放置方块(initialize block placement)
    def find_tag_init(self):
        self.y_stop = self.tag_y_stop
        self.head_track_state = 'track_tag'
        self.transport_state = 'place_block'
        # 更新接近停止的判断阈值，同上(update the determination threshold for approaching stop, same as above)
        # x方向位移误差阈值(像素)(threshold for x-axis displacement error (pixels))
        # y方向位移误差阈值(像素)(threshold for y-axis displacement error (pixels))
        # 航向角误差阈值(度)(threshold for yaw angle error (degrees))
        self.approach_object.update_approach_stop_value(10, 10, 5)
        self.walking_param['hip_pitch_offset'] = 5  # 髋关节角度偏移量(hip joint angle offset)
        # 更新步态参数(update gait parameter)
        self.approach_object.update_gait([400, 0.1, 0.03], 0, self.walking_param)
        self.head_move_position = copy.deepcopy(self.head_move_position_tag)
        rospy.ServiceProxy('/color_detection/stop', Empty)()
        rospy.ServiceProxy('/apriltag_ros/start', Empty)()

    # 通过其他apriltag判断目标apriltag位置(determine the position of the target AprilTag based on the position of other AprilTags)
    # apriltag摆放位置：红(tag36h11_1)，绿(tag36h11_2)，蓝(tag36h11_3)(AprilTag placement positions: red (tag36h11_1), green (tag36h11_2), and blue (tag36h11_3))
    def get_direction(self, target_tag, tag_data):
        # 目标tag， 当前检测到的tag(target tag; current detected tag)
        if target_tag == 1:  # 目标apriltag为1(if the target AprilTag is 1)
            if not tag_data['2']:  # 没有检测到apriltag 2(if AprilTag 2 is not detected)
                if tag_data['3']:  # 检测到apriltag 3， 则apriltag 1在apriltag 3左边，所以左转(If AprilTag 3 is detected, then AprilTag 1 is to the left of AprilTag 3. Turn left)
                    return 'left'
            else:                  # 检测到apriltag 2，则则apriltag 1在apriltag 2左边，所以左转(If AprilTag 2 is detected, then AprilTag 1 is to the left of AprilTag 2. Turn left)
                return 'left'
        elif target_tag == 2:
            if not tag_data['1']:
                if tag_data['3']:
                    return 'left'
            else:
                return 'right'
        elif target_tag == 3:
            if not tag_data['1']:
                if tag_data['2']:
                    return 'right'
            else:
                return 'right'

        return None
    
    # 头部PID跟踪(PID tracking for head)
    def head_track_process(self, x, y, width, height):
        # 头部追踪(head tracking)
        if abs(x - width/2) < 10:
            x = width/2
        if abs(y - height/2) < 10:
            y = height/2
        rl_dis = self.rl_track.track(x, width/2)
        ud_dis = self.ud_track.track(y, height/2)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

        return rl_dis, ud_dis
    
    # 躯体跟踪(body following)
    def body_track_process(self, rl_dis, ud_dis, object_data):
        self.object_info = None # 清空上一次的检测信息(clear the previous detection information)
        # 躯体追踪(body tracking)
        x, y, angle, width, height = object_data    # 获取检测到的目标信息(obtain the detected target information)
        self.rl_dis, self.ud_dis = None, None       # 清空头部pid计算的输出(clear the output calculated from head PID)
        if object_data is not None:
            # approach状态下进行跟踪(follow under the state of approach)
            if self.body_track_state == 'approach' and rl_dis is not None:
                self.approach_object.update_gait(step_mode=0)   # 更新步态为前进模式(update the gait to moving forward mode)
                # 更新步态移动范围(update the movement range of gait)
                self.approach_object.update_gait_range(y_range=[-0.01, 0.01], yaw_range=[-6, 6])
                # 计算旋转停止的目标位置(calculate target position for stopping rotation)
                yaw_stop = 500 + self.calib_config['head_pan_offset'] + math.copysign(self.yaw_stop, rl_dis - (500 + self.calib_config['head_pan_offset']))
                # 进行跟踪,如果接近目标则切换状态(Perform target tracking; if approaching to the target, it switches state)
                if self.approach_object.process(500 - ud_dis, 0 + self.calib_config['center_x_offset'], (yaw_stop - rl_dis)/9, self.head_tilt_range[0] - 40, 0, 0, width, height):
                    self.head_track_state = 'stop'
                    self.body_track_state = 'align'
                    time.sleep(0.5)
                    # 回正头部位置(return head to its initial position)
                    self.motion_manager.set_servos_position(200, [[23, self.head_pan_init], [24, self.head_tilt_init]])
                    time.sleep(0.5)
                    print('align')
            # align状态下进行旋转归中(rotate to the neutral position under the align state)
            elif self.body_track_state == 'align':
                if self.transport_state == 'pick_block':
                    if angle > 40:  # 调整角度，不取45，因为如果在45时值的不稳定会导致反复移动(Adjust the angle. Do not use 45 because unstable value may cause repeated movement)
                        angle -= 90
                elif self.transport_state == 'place_block':
                    self.walking_param['hip_pitch_offset'] = 5      # 更新步态参数(update gait parameter)
                    self.approach_object.update_gait([400, 0.1, 0.03], 0, self.walking_param)
                    if angle < 0:   # 调整角度(adjust the angle)
                        angle += 360
                    angle -= 180
                self.approach_object.update_gait(step_mode=1)   # 更新步态为旋转模式(update gait to rotation mode)
                # 更新步态移动范围(update movement range of gait)
                self.approach_object.update_gait_range(y_range=[-0.012, 0.012], yaw_range=[-4, 4])    
                # 进行跟踪,如果旋转归中则切换状态(Perform target tracking; if rotating to the neutral position, the state will be switched)
                if self.approach_object.process(y, x, angle, self.y_stop*width, self.block_x_stop*width, self.block_yaw_stop, width, height):
                    if self.transport_state == 'pick_block':     # 提起对象(pick up the object)
                        self.gait_manager.disable()
                        self.motion_manager.run_action(self.move_up_action_name)
                    elif self.transport_state == 'place_block':  # 放下对象(put down the object)
                        self.walking_param['hip_pitch_offset'] = 5
                        self.gait_manager.set_step([400, 0.2, 0.025], 0.01, 0, 0, self.walking_param, 0, 2) 
                        self.gait_manager.disable()
                        self.motion_manager.run_action(self.put_down_action_name)
                        self.walking_param['hip_pitch_offset'] = 15
                        self.gait_manager.set_step([400, 0.2, 0.025], -0.01, 0, 0, self.walking_param, 0, 7)
                    self.body_track_state = 'approach'
                    return True
                else:
                    time.sleep(0.8)
        return False

    # 搜索过程(the process of searching)
    def find_process(self):
        if rospy.get_time() > self.head_time_stamp:
            if self.start_index > len(self.head_move_position) - 1:
                self.start_index = 0
            rl_dis = self.head_move_position[self.start_index][0]
            ud_dis = self.head_move_position[self.start_index][1]
            self.rl_track.update_position(rl_dis)  # pid的输出值要跟着更新(update the output value of PID)
            self.ud_track.update_position(ud_dis)
            self.motion_manager.set_servos_position(self.head_move_position[self.start_index][2], [[23, rl_dis], [24, ud_dis]])
            if self.transport_state == 'place_block':
                self.walking_param['hip_pitch_offset'] = 5
                if self.get_direction(self.transport_dict[self.current_color], self.tag_data) == 'left':
                    self.gait_manager.set_step([400, 0.2, 0.025], 0, 0, 5, self.walking_param, 0, 0) 
                else:
                    self.gait_manager.set_step([400, 0.2, 0.025], 0, 0, -5, self.walking_param, 0, 0)   # 右转(turn right)
                self.tag_data = {'1': False, '2': False, '3': False}
            else:
                self.walking_param['hip_pitch_offset'] = 15
                self.gait_manager.set_step([400, 0.2, 0.025], 0, 0, -5, self.walking_param, 0, 0)
            self.head_time_stamp = rospy.get_time() + self.head_move_position[self.start_index][2]/1000.0
            self.start_index += 1
    
    # 主循环(main loop)
    def run(self):
        while self.running:
            if self.start:
                if self.object_info is not None:    # 如果检测到目标(if the target is detected)
                    if self.start_find_block:       # 需要搜寻的标志置为False(set the searching flag to False)
                        self.start_find_block = False
                    # 进行跟踪(perform the tracking)
                    elif self.body_track_process(self.rl_dis, self.ud_dis, self.object_info):
                        # 根据状态进行初始化(perform the initialization based on the state)
                        if self.transport_state == 'pick_block':    # 代表着去搬运方块的状态(represent the state for picking up the block)
                            self.find_tag_init()                            
                        elif self.transport_state == 'place_block': # 代表着去放置区放置方块的状态(represent the state for placing the block to the placement area)
                            self.find_block_init()
                    self.count_miss = 0  # 重置丢失次数(reset the miss count)
                else:                    # 如果未检测到目标(if the target is not detected)
                    if not self.start_find_block:
                        self.count_miss += 1        # 记录丢失次数(record the miss count)
                        if self.count_miss > 100:   # 如果超过阈值则置标志位开始搜寻(If the count exceeds the threshold, set the flag to start searching)
                            self.count_miss = 0
                            self.start_find_block = True
                            self.start_index = 0
                    else:
                        self.find_process()         # 搜寻过程(the finding process)
                time.sleep(0.01)
            else:
                time.sleep(0.01)

        self.init_action(self.head_pan_init, self.head_tilt_init)   # 停止头部动作(stop the action of head)
        self.stop_srv_callback(None)        # 停止检测(stop detection)
        rospy.signal_shutdown('shutdown')   # 关闭节点(close node)

if __name__ == "__main__":
    AutonomousTransportNode('autonomous_transport').run()
