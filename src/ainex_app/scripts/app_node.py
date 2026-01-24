#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/08/17
# @author:aiden
# APP Func
import os
import rospy
import numpy as np
from transitions import Machine
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Imu
from ainex_sdk.common import cv2_image2ros
import math, time, queue, threading, signal, os
from ainex_app.common import ColorPicker, Heart
from ainex_example.pid_track import PIDTrack
from ainex_sdk import pid, misc, common, voice_play
from ainex_example.approach_object import ApproachObject
from ainex_example.visual_patrol import VisualPatrol
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from ainex_interfaces.srv import SetWalkingCommand, GetWalkingState
from ainex_interfaces.srv import SetInt, SetIntRequest, SetIntResponse
from ainex_interfaces.srv import SetPoint, SetPointRequest, SetPointResponse
from ainex_interfaces.srv import SetFloat, SetFloatRequest, SetFloatResponse
from ainex_interfaces.msg import ColorDetect, ColorsDetect, ObjectsInfo
from ros_robot_controller.msg import BuzzerState, RGBState, RGBsState

# app
# 机体控制  回传，移动，动作，跌倒起来(robot control: live camera feed, movement, action, and getting up)
# 颜色识别(color recognition)
# 颜色追踪(color tracking)
# 智能巡线(intelligent line following)
# 自主踢球(autonomous ball kicking)
# 人脸识别(face recognition)
# 按键:巡线，踢球，常规, rgb指示(buttons: line following, ball kicking, normal, and RGB indicator)
class AppNode:
    # transitions各状态顺序(the order of each state)，trigger--->prepare--->condition--->before--->on_enter--->after--->next_prepare--->next_before--->on_exit--->next_on_enter
    states = [{'name': 'idle'}, 
              {'name': 'control', 'on_enter': 'on_enter_control', 'on_exit': 'on_exit_control'},
              {'name': 'visual_patrol', 'on_enter': 'on_enter_visual_patrol', 'on_exit': 'on_exit_visual_patrol'},
              {'name': 'color_detect', 'on_enter': 'on_enter_color_detect', 'on_exit': 'on_exit_color_detect'},
              {'name': 'color_track', 'on_enter': 'on_enter_color_track', 'on_exit': 'on_exit_color_track'},
              {'name': 'face_detect', 'on_enter': 'on_enter_face_detect', 'on_exit': 'on_exit_face_detect'},
              {'name': 'kick_ball', 'on_enter': 'on_enter_kick_ball', 'on_exit': 'on_exit_kick_ball'},
              {'name': 'fall_rise', 'on_enter': 'on_enter_fall_rise', 'on_exit': 'on_exit_fall_rise'},
              {'name': 'color_picker', 'on_enter': 'on_enter_color_picker', 'on_exit': 'on_exit_color_picker'}]

    transitions = [
            {'trigger': 'app_select_control', 'source': '*', 'dest': 'control', 'prepare': 'prepare_color_detect_image', 'before': 'control_init'},
            {'trigger': 'app_select_visual_patrol', 'source': '*', 'dest': 'visual_patrol', 'prepare': 'prepare_color_detect_image', 'before': 'visual_patrol_init'},
            {'trigger': 'app_select_color_detect', 'source': '*', 'dest': 'color_detect', 'prepare': 'prepare_color_detect_image', 'before': 'color_detect_init'},
            {'trigger': 'app_select_color_track', 'source': '*', 'dest': 'color_track', 'prepare': 'prepare_color_detect_image', 'before': 'color_track_init'},
            {'trigger': 'app_select_face_detect', 'source': '*', 'dest': 'face_detect', 'prepare': 'prepare_face_detect_image',  'before': 'face_detect_init'},
            {'trigger': 'app_select_kick_ball', 'source': '*', 'dest': 'kick_ball', 'prepare': 'prepare_color_detect_image', 'before': 'kick_ball_init'},
            {'trigger': 'app_select_fall_rise', 'source': '*', 'dest': 'fall_rise', 'prepare': 'prepare_imu', 'before': 'fall_rise_init'},
            {'trigger': 'app_select_color_picker', 'source': '*', 'dest': 'color_picker'},
            {'trigger': 'click_button_once', 'source': '*', 'dest': 'kick_ball', 'prepare': 'prepare_color_detect_image', 'before': 'kick_ball_init', 'after': 'kick_ball_start'},
            {'trigger': 'click_button_twice', 'source': '*', 'dest': 'visual_patrol', 'prepare': 'prepare_color_detect_image', 'before': 'visual_patrol_init', 'after': 'visual_patrol_start'},
            {'trigger': 'click_button_thrice', 'source': '*', 'dest': 'idle', 'before': 'idle_init'},
    ]
    
    image_process_size = [160, 120]

    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]

    circle_roi = [0, 1, 0, 1]

    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running = True
        self.button_func_on = False
        self.target_color = None
        self.color_picker = None
        self.is_running = False
        self.threshold = 0.2
        self.last_state = 'idle'
        self.objects_info = []
        self.language = os.environ['speaker_language']
        self.image_queue = queue.Queue(maxsize=2)
        self.lock = threading.RLock()  # 线程互斥锁(Thread mutex lock)
       
        self.head_init_pose = {'control':  [[500, 500], [125, 875], [315, 625]], 
                                'visual_patrol': [[500, 260]],
                                'color_detect': [[500, 550]],
                                'color_track': [[500, 500], [125, 875], [315, 625]],
                                'face_detect': [[500, 630]],
                                'kick_ball': [[500, 300], [125, 875], [265, 500]],
                                'fall_rise': [[500, 500]]}
        
        
        self.calib_config = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_example/config/calib.yaml')
        # 机器人行走的库调用(call the library for robot walking)
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        signal.signal(signal.SIGINT, self.shutdown)

        #########color_track init##########
        self.color_detect_image_sub = None
        self.config = rospy.get_param('color_track')
        self.color_track_pid_rl = pid.PID(self.config['pid1_p'], self.config['pid1_i'], self.config['pid1_d'])
        self.color_track_pid_ud = pid.PID(self.config['pid2_p'], self.config['pid2_i'], self.config['pid2_d'])
        
        self.color_track_rl_track = PIDTrack(self.color_track_pid_rl, self.head_init_pose['color_track'][1], self.head_init_pose['color_track'][0][0])
        self.color_track_ud_track = PIDTrack(self.color_track_pid_ud, self.head_init_pose['color_track'][2], self.head_init_pose['color_track'][0][1])
        #########kick_ball##########################
        self.head_time_stamp = rospy.get_time()
        self.count_miss = 0
        # 找球相关标志位(relevant flags for finding the ball)
        self.start_index = 0
        self.start_find_ball = False
        
        # 存放检测结果(store detection result)
        self.rl_dis = None
        self.ud_dis = None

        # 左右踢球的动作名(action names for kicking the ball left and right)
        self.left_shot_action_name = 'left_shot'
        self.right_shot_action_name = 'right_shot'
        self.pid_rl = pid.PID(0.1, 0.0, 0.001)
        self.pid_ud = pid.PID(0.1, 0.0, 0.001)
        self.head_pan_init = self.head_init_pose['kick_ball'][0][0]   # 左右舵机的初始值(the initial value of left and right servos)
        self.head_tilt_init = self.head_init_pose['kick_ball'][0][1]  # 上下舵机的初始值(the initial value of up and down servos)
        self.head_pan_range = self.head_init_pose['kick_ball'][1]
        self.head_tilt_range = self.head_init_pose['kick_ball'][2]
        self.rl_track = PIDTrack(self.pid_rl, self.head_pan_range, self.head_pan_init)
        self.ud_track = PIDTrack(self.pid_ud, self.head_tilt_range, self.head_tilt_init)
        
        # 躯体的追踪参数(the tracking parameter of the body)
        self.yaw_stop = 40  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值(The difference between the pulse width value of the left and right servo motors of the head when the body stops rotating and the neutral position of 500)

        self.approach_object = ApproachObject(self.gait_manager, step_mode=0)
        self.approach_object.update_gait(dsp=[400, 0.2, 0.02])
        self.approach_object.update_stop_count(1)
        self.approach_object.update_gait_range(x_range=[-0.013, 0.013])
        self.approach_object.update_approach_stop_value(30, 0, 3)
        # 找球时头部经过的5个位置，左右舵机，上下舵机，时间ms(When searching for the ball, the robot head will pass through 5 positions, with specific pulse width values and time (in milliseconds) for the left-right and up-down servos at each position)
        # left_down, left_up, center_up, right_up, right_down
        self.find_ball_position = [[650, 300, 1000], 
                                   [650, 500, 1000], 
                                   [500, 500, 1000], 
                                   [350, 500, 1000],
                                   [350, 300, 1000]
                                   ]
        #########visual_patrol##########################
        # 创建巡线控制实例(create line following control instance)
        self.visual_patrol = VisualPatrol(self.gait_manager)
        #########face_detect##########################
        self.face_detect_image_sub = None
        self.d_pulse = 5
        self.head_init = self.head_init_pose['face_detect'][0][0]
        #########fall_rise##########################
        # 开启imu发布(start IMU publishing)
        self.imu_sub = None 
        self.robot_state = 'stand'
        # 前后倒计数(Countdown forwards and backwards)
        self.count_lie = 0
        self.count_recline = 0
        self.lie_to_stand_action_name = 'lie_to_stand'          # 向前倒起立动作名(Action name for standing up from lying forward)
        self.recline_to_stand_action_name = 'recline_to_stand'  # 向后倒起立动作名(Action name for standing up from lying backward)
        self.timeout = 6
        self.start_time = time.time()
        ###########button################
        self.button_selected = False
        self.delay_before_start = 0
        self.button_state = 0
        self.button_pressed = False
        self.count_button_press = 0
        self.count_button_release = 0
        rospy.wait_for_service('/sensor/button/enable')
        # 开启按钮发布(publish start button)
        rospy.ServiceProxy('/sensor/button/enable', SetBool)(True)
        # 订阅按钮状态(subscribe to button state)
        rospy.Subscriber('/sensor/button/get_button_state', Bool, self.button_callback)
        ###############################

        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)   # 发布颜色检测类型(publish color detection type)
        self.result_publisher = rospy.Publisher(self.name + '/image_result', Image, queue_size=1)  # 发布图像(publish image)
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
        rospy.Service(self.name + '/enter', SetInt,  self.enter_srv_callback)  # 进入玩法(enter game)
        rospy.Service(self.name + '/set_running', SetBool, self.set_running_srv_callback)  # 开启/关闭 玩法(start/exit game)
        rospy.Service(self.name + '/set_target_color', SetPoint, self.set_target_color_srv_callback)  # 设置追踪颜色(set color to be tracked)
        rospy.Service(self.name + '/get_target_color', Trigger, self.get_target_color_srv_callback)  # 获取追踪颜色(obtain color to be tracked)
        rospy.Service(self.name + '/set_threshold', SetFloat, self.set_threshold_srv_callback)  # 设置阈值(set threshold)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.heartbeat_timeout_callback())  # 心跳(heartbeat)
        
        time.sleep(0.2)
        self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
        self.rgb_pub.publish(RGBsState([RGBState(1, 0, 255, 0)]))
        self.machine = Machine(model=self, states=self.states, transitions=self.transitions, initial='idle',
                               ignore_invalid_triggers=True, queued=True)
        voice_play.play('running', language=self.language)
        
    # 退出回调(exit callback)
    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    def init_action(self, head_pan_init, head_tilt_init, delay=200):
        # 初始位置(initial position)
        rospy.ServiceProxy('/walking/init_pose', Empty)()
        self.motion_manager.set_servos_position(delay, [[23, head_pan_init], [24, head_tilt_init]])

    def heartbeat_timeout_callback(self):
        rospy.loginfo('heartbeat timeout')

    def on_enter_control(self):
        rospy.loginfo('on_enter_control')

    def on_exit_control(self):
        rospy.loginfo('on_exit_control')

    def on_enter_visual_patrol(self):
        rospy.loginfo('on_enter_visual_patrol')
        self.is_running = False

    def on_exit_visual_patrol(self):
        self.gait_manager.stop()
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        rospy.loginfo('on_exit_visual_patrol')

    def on_enter_color_detect(self):
        rospy.loginfo('on_enter_color_detect')
        self.is_running = False

    def on_exit_color_detect(self):
        self.gait_manager.stop()
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        rospy.loginfo('on_exit_color_detect')

    def on_enter_color_track(self):
        rospy.loginfo('on_enter_color_track')
        self.is_running = False

    def on_exit_color_track(self):
        self.gait_manager.stop()
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        rospy.loginfo('on_exit_color_track')

    def on_enter_face_detect(self):
        rospy.loginfo('on_enter_face_detect')
        self.is_running = False

    def on_exit_face_detect(self):
        self.gait_manager.stop()
        rospy.ServiceProxy('/face_detect/exit', Empty)()
        rospy.loginfo('on_exit_face_detect')

    def on_enter_kick_ball(self):
        rospy.loginfo('on_enter_kick_ball')
        self.is_running = False
        self.head_time_stamp = rospy.get_time()
        self.count_miss = 0
        # 找球相关标志位(relevant flag for finding the ball)
        self.start_index = 0
        self.start_find_ball = False
        
        # 存放检测结果(store the detection result)
        self.rl_dis = None
        self.ud_dis = None

    def on_exit_kick_ball(self):
        self.gait_manager.stop()
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        rospy.loginfo('on_exit_kick_ball')
   
    def on_enter_fall_rise(self):
        rospy.loginfo('on_enter_fall_rise')

    def on_exit_fall_rise(self):
        rospy.loginfo('on_exit_fall_rise')

    def on_enter_color_picker(self):
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        rospy.loginfo('on_enter_color_picker')

    def on_exit_color_picker(self):
        rospy.loginfo('on_exit_color_picker')

    def idle_init(self):
        rospy.loginfo('idle_init')
        self.button_func_on = False
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable_control')
        try:
            if self.face_detect_image_sub is not None:
                rospy.ServiceProxy('/face_detect/exit', Empty)()
                self.face_detect_image_sub.unregister()
            if self.color_detect_image_sub is not None:
                rospy.ServiceProxy('/color_detection/exit', Empty)()
                self.color_detect_image_sub.unregister()
            if self.imu_sub is not None:
                self.imu_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        self.is_running = False

    def control_init(self):
        rospy.loginfo('control_init')
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable_control')
        self.init_action(self.head_init_pose['control'][0][0], self.head_init_pose['control'][0][1])

    def visual_patrol_init(self):
        rospy.loginfo('visual_patrol_init')
        self.init_action(self.head_init_pose['visual_patrol'][0][0], self.head_init_pose['visual_patrol'][0][1])

    def color_detect_init(self):
        rospy.loginfo("color_detect_init")
        self.init_action(self.head_init_pose['color_detect'][0][0], self.head_init_pose['color_detect'][0][1])

    def color_track_init(self):
        self.init_action(self.head_init_pose['color_track'][0][0], self.head_init_pose['color_track'][0][1])

    def face_detect_init(self):
        rospy.loginfo("face_detect_init")
        self.init_action(self.head_init_pose['face_detect'][0][0], self.head_init_pose['face_detect'][0][1])

    def kick_ball_init(self):
        rospy.loginfo("kick_ball_init")
        self.init_action(self.head_init_pose['kick_ball'][0][0], self.head_init_pose['kick_ball'][0][1])

    def fall_rise_init(self):
        rospy.loginfo("fall_rise_init")
        self.init_action(self.head_init_pose['fall_rise'][0][0], self.head_init_pose['fall_rise'][0][1])
        time.sleep(1)
        while True:
            res = rospy.ServiceProxy('/walking/is_walking', GetWalkingState)()
            if not res.state:
                break
            time.sleep(0.1)
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('disable_control')

    def kick_ball_start(self):
        rospy.loginfo("kick_ball_start")
        self.button_func_on = True
        self.set_circle_color(color_name='blue')
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        rospy.ServiceProxy('/color_detection/start', Empty)()  # 开启颜色识别(start color recognition)
        self.is_running = True

    def visual_patrol_start(self):
        rospy.loginfo("visual_patrol_start")
        self.button_func_on = True
        self.set_line_color(color_name='black')
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        rospy.ServiceProxy('/color_detection/start', Empty)()  # 开启颜色识别(start color recognition)
        self.is_running = True

    def prepare_color_detect_image(self):
        rospy.loginfo("prepare_color_detect_image")
        self.color_detect_image_sub = rospy.Subscriber('/color_detection/image_result', Image, self.color_detect_image_callback, queue_size=1)  # 订阅颜色检测画面(subscribe to color detection image)
        self.target_color = None
        self.threshold = 0.2
        self.color_picker = None
        self.objects_info = []
        self.is_running = False
        rospy.ServiceProxy('/color_detection/enter', Empty)()  # 开启颜色识别节点(start color recognition node)

    def prepare_face_detect_image(self):
        self.face_detect_image_sub = rospy.Subscriber('/face_detect/image_result', Image, self.face_detect_image_callback, queue_size=1)
        rospy.loginfo("prepare_face_detect_image")
        self.objects_info = []
        self.is_running = False
        rospy.ServiceProxy('/face_detect/enter', Empty)()  # 开启人脸检测节点(start face detection node)

    def prepare_imu(self):
        rospy.loginfo("prepare_imu")
        try:
            if self.imu_sub is not None:
                self.imu_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        self.robot_state = 'stand'
        # 前后倒计数(countdown forwards and backwards)
        self.count_lie = 0
        self.count_recline = 0
        self.start_time = time.time()
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))

    def enter_srv_callback(self, msg):
        rospy.loginfo("%s enter"%msg.data)
        if msg.data == 0:
            self.click_button_thrice()
        elif msg.data == 1:
            self.app_select_control()
        elif msg.data == 2:
            self.app_select_kick_ball()
        elif msg.data == 3:
            self.app_select_color_detect()
        elif msg.data == 4:
            self.app_select_visual_patrol()
        elif msg.data == 5:
            self.app_select_color_track()
        elif msg.data == 6:
            self.app_select_face_detect()
        elif msg.data == 7:
            self.app_select_fall_rise()
        return SetIntResponse(success=True) 

    def set_target_color_srv_callback(self, req: SetPointRequest):
        rospy.loginfo("set_target_color")
        with self.lock:
            x, y = req.data.x, req.data.y
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.last_state = self.state
                self.color_picker = ColorPicker(req.data, 5)
                self.app_select_color_picker()
        return SetPointResponse(success=True)

    def get_target_color_srv_callback(self, _):
        rospy.loginfo("get_target_color")
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.target_color is not None:
                rsp.success = True
                rgb = self.target_color[1]
                print('rgb', rgb)
                rsp.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return rsp

    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        with self.lock:
            if not req.data:
                self.is_running = req.data
                self.gait_manager.stop()

                if self.state == 'color_detect':
                    rospy.ServiceProxy('/color_detection/stop', Empty)()
                elif self.state == 'face_detect':
                    rospy.ServiceProxy('/face_detect/stop', Empty)()
            else:
                if self.state == 'idle':
                    return SetBoolResponse(success=False)
                else:
                    self.init_action(self.head_init_pose[self.state][0][0], self.head_init_pose[self.state][0][1])
                    self.is_running = req.data
                    if self.state == 'color_detect':
                        rospy.ServiceProxy('/color_detection/start', Empty)()
                        self.set_colors()
                    elif self.state == 'face_detect':
                        rospy.ServiceProxy('/face_detect/start', Empty)()
        return SetBoolResponse(success=req.data)

    def set_threshold_srv_callback(self, req: SetFloatRequest):
        rospy.loginfo("set threshold")
        with self.lock:
            self.threshold = req.data
            if self.target_color is not None:
                min_color = [int(self.target_color[0][0] - 50 * self.threshold * 2), 
                             int(self.target_color[0][1] - 50 * self.threshold),
                             int(self.target_color[0][2] - 50 * self.threshold)]
                max_color = [int(self.target_color[0][0] + 50 * self.threshold * 2), 
                             int(self.target_color[0][1] + 50 * self.threshold), 
                             int(self.target_color[0][2] + 50 * self.threshold)]
                self.set_color(min_color, max_color)
            return SetFloatResponse(success=True)

    def set_color(self, min_color, max_color):
        if self.state == 'visual_patrol':
            self.set_line_color(min_color, max_color)
        else:
            self.set_circle_color(min_color, max_color)

    def set_colors(self):
        rospy.loginfo("set_colors")
        red_param = ColorDetect()
        red_param.use_name = True
        red_param.color_name = 'red'
        red_param.detect_type = 'circle'
        red_param.image_process_size = self.image_process_size
        red_param.min_area = self.image_process_size[0]*self.image_process_size[1] / 16 / 16
        red_param.max_area = self.image_process_size[0]*self.image_process_size[1]

        green_param = ColorDetect()
        green_param.use_name = True
        green_param.color_name = 'green'
        green_param.detect_type = 'circle'
        green_param.image_process_size = self.image_process_size
        green_param.min_area = self.image_process_size[0]*self.image_process_size[1] / 16 / 16
        green_param.max_area = self.image_process_size[0]*self.image_process_size[1]

        blue_param = ColorDetect()
        blue_param.use_name = True
        blue_param.color_name = 'blue'
        blue_param.detect_type = 'circle'
        blue_param.image_process_size = self.image_process_size
        blue_param.min_area = self.image_process_size[0]*self.image_process_size[1] / 16 / 16
        blue_param.max_area = self.image_process_size[0]*self.image_process_size[1]

        self.detect_pub.publish([red_param, green_param, blue_param])

    def set_circle_color(self, min_color=None, max_color=None, color_name=None):
        rospy.loginfo("set_circle_color")
        param = ColorDetect()
        if color_name is not None:
            param.use_name = True
            param.color_name = color_name
        else:
            param.use_name = False
            param.lab_min = min_color
            param.lab_max = max_color
        param.detect_type = 'circle'
        param.image_process_size = self.image_process_size
        param.min_area = 20
        param.max_area = self.image_process_size[0]*self.image_process_size[1]
        self.detect_pub.publish([param])

        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def set_line_color(self, min_color=None, max_color=None, color_name=None):
        rospy.loginfo("set_line_color")
        # 生成颜色识别参数(generate color recognition parameter)
        param = ColorDetect()
        if color_name is not None:
            param.use_name = True
            param.color_name = color_name
        else:
            param.use_name = False
            param.lab_min = min_color
            param.lab_max = max_color
        param.detect_type = 'line'  # 指定了检测类型为线条"line"(specify the detection type as the "line")
        param.image_process_size = self.image_process_size
        # 设置ROI参数(set ROI parameter)
        param.line_roi.up.y_min = int(self.line_roi[0][0] * self.image_process_size[1])
        param.line_roi.up.y_max = int(self.line_roi[0][1] * self.image_process_size[1])
        param.line_roi.up.x_min = int(self.line_roi[0][2] * self.image_process_size[0])
        param.line_roi.up.x_max = int(self.line_roi[0][3] * self.image_process_size[0])

        param.line_roi.center.y_min = int(self.line_roi[1][0] * self.image_process_size[1])
        param.line_roi.center.y_max = int(self.line_roi[1][1] * self.image_process_size[1])
        param.line_roi.center.x_min = int(self.line_roi[1][2] * self.image_process_size[0])
        param.line_roi.center.x_max = int(self.line_roi[1][3] * self.image_process_size[0])

        param.line_roi.down.y_min = int(self.line_roi[2][0] * self.image_process_size[1])
        param.line_roi.down.y_max = int(self.line_roi[2][1] * self.image_process_size[1])
        param.line_roi.down.x_min = int(self.line_roi[2][2] * self.image_process_size[0])
        param.line_roi.down.x_max = int(self.line_roi[2][3] * self.image_process_size[0])
        # 面积过滤参数(area filtering parameter)
        param.min_area = 1
        param.max_area = self.image_process_size[0] * self.image_process_size[1]
        # 发布颜色识别参数(publish color recognition parameter)
        self.detect_pub.publish([param])
        common.loginfo('%s set_color' % self.name)
        return [True, 'set_color']

    def color_detect_image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        if self.state == 'color_picker':
            if self.image_queue.full():
                self.image_queue.get()
            self.image_queue.put(rgb_image)
        else:
            # cv2.cvtColor 更占cpu(take up CPU)
            self.result_publisher.publish(cv2_image2ros(rgb_image[:, :, ::-1]))  # 发布图像(publish image)
        self.color_sub_connect = True

    def face_detect_image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
        self.result_publisher.publish(cv2_image2ros(rgb_image[:, :, ::-1]))  # 发布图像(publish image)

    # IMU回调函数(IMU callback function)
    def imu_callback(self, msg):
        #转化为角度值(convert to angle value)
        angle = abs(int(math.degrees(math.atan2(msg.linear_acceleration.y, msg.linear_acceleration.z)))) 
        # 根据状态计数(count based on the state)
        if self.robot_state == 'stand':
            if angle < 30:
                self.count_lie += 1
            else:
                self.count_lie = 0
            if angle > 150:
                self.count_recline += 1
            else:
                self.count_recline = 0
            # 根据计数判断是否进入起立状态(determine whether to access the standing up state based on the count)
            if self.count_lie > 50:
                self.count_lie = 0
                self.robot_state = 'lie_to_stand'
            elif self.count_recline > 50:
                self.count_recline = 0
                self.robot_state = 'recline_to_stand'
            time.sleep(0.02)

    def button_callback(self, msg):
        # rospy.loginfo('button_state: %s'%msg.data)
        if msg.data == 0:
            self.count_button_release = 0
            self.count_button_press += 1
        else:
            if self.button_pressed:
                self.count_button_release += 1
            self.count_button_press = 0
        if self.button_pressed:
            if self.count_button_release >= 5:
                self.count_button_release = 0
                self.count_button_press = 0
                self.button_pressed = False
                self.button_state += 1
                if self.button_state == 1:
                    self.is_running = False
                    self.gait_manager.stop()
                    self.rgb_pub.publish(RGBsState([RGBState(1, 0, 0, 255)]))
                    self.delay_before_start = time.time()
                    self.button_selected = True
                elif self.button_state == 2:
                    self.is_running = False
                    self.gait_manager.stop()
                    self.rgb_pub.publish(RGBsState([RGBState(1, 255, 0, 0)]))
                    self.delay_before_start = time.time()
                    self.button_selected = True
                elif self.button_state == 3:
                    self.rgb_pub.publish(RGBsState([RGBState(1, 0, 255, 0)]))
                    self.click_button_thrice()
                    self.button_state = 0
            elif self.count_button_press > 200:
                self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.5, off_time=0.01, repeat=1))
                time.sleep(0.5)
                self.count_button_press = 0
                self.count_button_release = 0
                with open('/home/ubuntu/share/src/.halt.txt', 'w') as f:
                    f.write('1')
                # os.system('sudo halt')
                time.sleep(2)
        else:
            if self.count_button_press >= 10:
                self.count_button_press = 10
                self.button_pressed = True
                # 蜂鸣器提示(buzzer sounds)
                self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
        if self.button_selected:
            if time.time() - self.delay_before_start > 2:
                self.button_selected = False
                if self.button_state == 1:
                    self.click_button_once()
                elif self.button_state == 2:
                    self.click_button_twice()

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data 
        if self.state == 'kick_ball' and self.is_running:
            for object_info in msg.data:
                if object_info.type == 'circle':
                    object_info.x = object_info.x - self.calib_config['center_x_offset']
                    self.rl_dis, self.ud_dis = self.head_track_process(object_info)

    def color_picker_process(self, rgb_image):
        with self.lock:
            if self.color_picker is not None:  # 拾取器存在(color picker exits)
                self.target_color, result_image = self.color_picker.get_color(rgb_image, rgb_image.copy())
                if self.target_color is not None:
                    # 计算检测颜色的阈值并发布(calculate and detect color threshold to publish)
                    
                    self.color_picker = None
                    min_color = [int(self.target_color[0][0] - 50 * self.threshold * 2), 
                                 int(self.target_color[0][1] - 50 * self.threshold),
                                 int(self.target_color[0][2] - 50 * self.threshold)]
                    max_color = [int(self.target_color[0][0] + 50 * self.threshold * 2), 
                                 int(self.target_color[0][1] + 50 * self.threshold), 
                                 int(self.target_color[0][2] + 50 * self.threshold)]
                    return [min_color, max_color], result_image
                else:
                    return None, result_image
            else:
                return None, rgb_image

    def color_detect_process(self, object_info):
        if object_info is not None:
            if object_info.label == 'green' or object_info.label == 'blue':
                self.motion_manager.set_servos_position(200, [[23, 600], [24, 550]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 400], [24, 550]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 600], [24, 550]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 400], [24, 550]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 550]])
                time.sleep(2)
            elif object_info.label == 'red':
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 650]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 450]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 650]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 450]])
                time.sleep(0.2)
                self.motion_manager.set_servos_position(200, [[23, 500], [24, 550]])
                time.sleep(2)

    # 头部PID跟踪(head PID tracking)
    def head_track_process(self, object_info):
        # 头部追踪(head tracking)
        if abs(object_info.x - object_info.width/2) < 10:
            object_info.x = object_info.width/2
        if abs(object_info.y - object_info.height/2) < 10:
            object_info.y = object_info.height/2
        rl_dis = self.rl_track.track(object_info.x, object_info.width/2)
        ud_dis = self.ud_track.track(object_info.y, object_info.height/2)
        self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

        return rl_dis, ud_dis
    
    # 躯体追踪(body tracking)
    def body_track_process(self, rl_dis, ud_dis, object_info):                                                
        # 左右根据头部左右舵机位置值进行调整(adjust direction based on the position value of left and right head servos)
        if object_info is not None:
            yaw_stop = 500 + math.copysign(self.yaw_stop, rl_dis - 500)
            yaw_stop_ = (yaw_stop - rl_dis)/9
            if 10 < abs(yaw_stop - rl_dis) < 27:
                yaw_stop_ = math.copysign(3, yaw_stop - rl_dis)
            if self.approach_object.process(500 - ud_dis, 0 + self.calib_config['center_x_offset'], yaw_stop_, self.head_tilt_range[0], 0, 0, object_info.width, object_info.height):
                self.gait_manager.disable()
                if rl_dis > 500:
                    self.motion_manager.run_action(self.left_shot_action_name)  # 左脚踢(kicking ball with the left foot)
                else:
                    self.motion_manager.run_action(self.right_shot_action_name)  # 右脚踢(kicking ball with the right foot)
    # 找球过程(the process of finding the ball)
    def find_ball_process(self):
        # 根据预设的扫描点进行找球(find the ball based on the preset scanning points)
        if rospy.get_time() > self.head_time_stamp:
            if self.start_index > len(self.find_ball_position) - 1:
                self.start_index = 0
            rl_dis = self.find_ball_position[self.start_index][0]
            ud_dis = self.find_ball_position[self.start_index][1]
            self.rl_track.update_position(rl_dis)  # pid的输出值要跟着更新(update the output value of PID)
            self.ud_track.update_position(ud_dis)
            self.motion_manager.set_servos_position(self.find_ball_position[self.start_index][2], [[23, rl_dis], [24, ud_dis]])

            self.gait_manager.move(2, 0, 0, 5)  # 右转(turning right)
            self.head_time_stamp = rospy.get_time() + self.find_ball_position[self.start_index][2]/1000.0
            self.start_index += 1

    def kick_ball_process(self, object_info):
        # 如果识别到球就进行躯体追踪(If the ball is recognized, perform body tracking)
        if self.rl_dis is not None:
            self.body_track_process(self.rl_dis, self.ud_dis, object_info)
            self.rl_dis = None
            self.count_miss = 0
            self.start_find_ball = False
        # 如果未识别到球(If no ball is recognized)
        else:
            # 如果未开始找球(if it does not start to find the ball)
            if not self.start_find_ball:
                self.count_miss += 1    # 未识别计数加1(If no ball is recognized, increase the count by 1)
                 # 超过阈值则开始找球(If the count is greater than the threshold, it starts to find the ball)
                if self.count_miss > 100:
                    self.count_miss = 0
                    self.start_find_ball = True
                    self.start_index = 0
                time.sleep(0.01)
            # 如果已开始找球(if it starts finding the ball)
            else:
                self.find_ball_process()   # 进行找球过程(the process of finding the ball)

    def color_track_process(self, object_info):
        if object_info is not None:
            if abs(object_info.x - object_info.width/2) < 20:
                object_info.x = object_info.width/2
            if abs(object_info.y - object_info.height/2) < 20:
                object_info.y = object_info.height/2
            rl_dis = self.color_track_rl_track.track(object_info.x, object_info.width/2)
            ud_dis = self.color_track_ud_track.track(object_info.y, object_info.height/2)
            self.motion_manager.set_servos_position(20, [[23, int(rl_dis)], [24, int(ud_dis)]])

    def face_detect_process(self, object_info):
        if object_info is not None:
            if abs(object_info.x - object_info.width/2) < 50:
                while True:
                    res = rospy.ServiceProxy('/walking/is_walking', GetWalkingState)()
                    if not res.state:
                        break
                    else:
                        time.sleep(0.1)
                self.motion_manager.run_action('greet')
                time.sleep(1)
            else:
                if self.head_init > 800 or self.head_init < 200:
                    self.d_pulse = -self.d_pulse
                self.head_init += self.d_pulse
                self.motion_manager.set_servos_position(50, [[23, self.head_init]])
                time.sleep(0.05)
        else:
            if self.head_init > 800 or self.head_init < 200:
                self.d_pulse = -self.d_pulse
            self.head_init += self.d_pulse
            self.motion_manager.set_servos_position(50, [[23, self.head_init]])
            time.sleep(0.05)

    def fall_rise_process(self):
        if time.time() - self.start_time > self.timeout:
            self.robot_state = 'stand'
            
            self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.2, off_time=0.01, repeat=1))
            self.app_select_control()

        # 如果不是站立状态,进行起立(If not in the standing up state, it stands up)
        if self.robot_state != 'stand':
            # 蜂鸣器提示(the buzzer sounds)
            self.buzzer_pub.publish(BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1))
            self.gait_manager.disable()  # 停止步态(disable gait)
            # 根据状态选择起立动作(select standing up action based on the state)
            if self.robot_state == 'lie_to_stand':
                common.loginfo('lie_to_stand')
                self.motion_manager.run_action(self.lie_to_stand_action_name)
                self.app_select_control()
            elif self.robot_state == 'recline_to_stand':
                common.loginfo('recline_to_stand')
                self.motion_manager.run_action(self.recline_to_stand_action_name)
                self.app_select_control()
            time.sleep(0.5)
            self.robot_state = 'stand'  # 起立后切换到站立状态(After standing up, it switches to the standing up state)
        else:
            time.sleep(0.01)

    def run(self):
        while self.running:
            if self.state == 'idle':
                time.sleep(0.01)
            else:
                time_start = time.time()
                
                if self.state == 'control':
                    pass
                elif self.state == 'color_picker':
                    image = self.image_queue.get(block=True)
                    res, result_image = self.color_picker_process(image)
                    if res is not None:
                        rospy.ServiceProxy('/color_detection/start', Empty)()  # 开启颜色识别(enable color recognition)
                        self.state = self.last_state
                        self.set_color(res[0], res[1])
                    # cv2.cvtColor 更占cpu(take up CPU)
                    self.result_publisher.publish(cv2_image2ros(result_image[:, :, ::-1]))  # 发布图像(publish image)
                else:
                    if self.is_running:
                        # 获取识别结果(obtain recognition result)
                        line_data = None
                        circle_data = None
                        face_data = None
                        
                        for object_info in self.objects_info:
                            if object_info.type == 'line':
                                line_data = object_info
                            elif object_info.type == 'circle':
                                circle_data = object_info
                            elif object_info.label == 'face':
                                face_data = object_info
                        self.objects_info = []
                        if self.state == 'kick_ball' and (self.target_color is not None or self.button_func_on):
                            self.kick_ball_process(circle_data)
                        elif self.state == 'color_detect':
                            self.color_detect_process(circle_data)
                        elif self.state == 'visual_patrol' and (self.target_color is not None or self.button_func_on): 
                            if line_data is not None:
                                self.visual_patrol.process(line_data.x, line_data.width)
                        elif self.state == 'color_track' and self.target_color is not None:
                            self.color_track_process(circle_data)
                        elif self.state == 'face_detect':
                            self.face_detect_process(face_data)

                    if self.state == 'fall_rise':
                        self.fall_rise_process()
                time_d = 0.03 - (time.time() - time_start)
                if time_d > 0:
                    time.sleep(time_d)

        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    AppNode('app').run()
