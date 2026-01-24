#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden
import time
import copy
import math
import rospy
from ainex_sdk import common
from sensor_msgs.msg import JointState
from ainex_kinematics.kinematics import LegIK
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64, String, Bool
from ainex_kinematics.walking_module import WalkingModule
from ainex_interfaces.msg import WalkingParam, AppWalkingParam, HeadState
from ainex_interfaces.srv import SetWalkingParam, GetWalkingParam, SetWalkingCommand, GetWalkingState

class JointPosition:
    def __init__(self):
        self.present_position = 0.0
        self.goal_position = 0.0

class Controller:
    RADIANS_PER_ENCODER_TICK = 240 * 3.1415926 / 180 / 1000
    ENCODER_TICKS_PER_RADIAN = 180 / 3.1415926 / 240 * 1000
    joint_index = {'r_hip_yaw':   0,
                   'r_hip_roll':  1,
                   'r_hip_pitch': 2,
                   'r_knee':      3,
                   'r_ank_pitch': 4,
                   'r_ank_roll':  5,
                   'l_hip_yaw':   6,
                   'l_hip_roll':  7,
                   'l_hip_pitch': 8,
                   'l_knee':      9,
                   'l_ank_pitch': 10,
                   'l_ank_roll':  11,
                   'r_sho_pitch': 12,
                   'l_sho_pitch': 13}
    
    # 舵机关节名称和舵机id的对应关系(servo joint names and their corresponding servo IDs)
    joint_id = {'r_hip_yaw':   12,
                'r_hip_roll':  10,
                'r_hip_pitch': 8,
                'r_knee':      6,
                'r_ank_pitch': 4,
                'r_ank_roll':  2,
                'l_hip_yaw':   11,
                'l_hip_roll':  9,
                'l_hip_pitch': 7,
                'l_knee':      5,
                'l_ank_pitch': 3,
                'l_ank_roll':  1,
                'r_sho_pitch': 14,
                'l_sho_pitch': 13,
                'l_sho_roll':  15,
                'r_sho_roll':  16,
                'l_el_pitch':  17,
                'r_el_pitch':  18,
                'l_el_yaw':    19,
                'r_el_yaw':    20,
                'l_gripper':   21,
                'r_gripper':   22,
                'head_pan':    23,
                'head_tilt':   24}

    joint_name = {value: key for key, value in joint_id.items()}

    # 躯体可以比完全站直低的范围，单位m(the range of the body height for standing up, in units of m)
    body_height_range = [0.015, 0.06] 
    # 步幅范围，单位m(the range of stride, in units of m)
    x_amplitude_range = [-0.05, 0.05]
    y_amplitude_range = [-0.05, 0.05]
    # 腿抬高范围，单位m(the range of height for lifting legs, in units of m)
    step_height_range = [0.0, 0.05]
    # 转弯范围，单位度(the range of turning, in units of degrees)
    angle_amplitude_range = [-10, 10]
    # 手臂摆动范围，单位弧度(the range of arm swing, in units of radians)
    arm_swap_range = [0, math.radians(60)]

    y_swap_range = [0, 0.05]

    def __init__(self, name):
        rospy.init_node(name)
        rospy.set_param('/init_pose/init_finish', False)
        self.gazebo_sim = rospy.get_param('~gazebo_sim', False)
        if self.gazebo_sim:
            self.init_pose_finish = False
        else:
            self.init_pose_finish = True
        ####################舵机脉宽和弧度的关系(the relationship between the servo pulse width and the radian)#############
        self.joint_angles_convert_coef = {}
        items_ = rospy.get_param('~controllers').items()
        for ctl_name, ctl_params in items_:
            if ctl_params['type'] == 'JointPositionController':
                initial_position_raw = ctl_params['servo']['init']
                min_angle_raw = ctl_params['servo']['min']
                max_angle_raw = ctl_params['servo']['max']
                flipped = min_angle_raw > max_angle_raw

                if flipped:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, -self.ENCODER_TICKS_PER_RADIAN]
                else:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, self.ENCODER_TICKS_PER_RADIAN]
        #########初始姿态(initial posture)#########
        self.init_pose_data = rospy.get_param('~init_pose')
        self.init_servo_data = []
        for joint_name in self.init_pose_data:
            id_ = self.joint_id[joint_name]
            angle = self.init_pose_data[joint_name]
            pulse = self.angle2pulse(id_, angle)
            self.init_servo_data.extend([[id_, pulse]])
        if not self.gazebo_sim: 
            from ainex_kinematics.motion_manager import MotionManager
            self.motion_manager = MotionManager()
            self.motion_manager.set_servos_position(1000, self.init_servo_data)
            time.sleep(1)
        #########################

        self.walking_enable = True
        self.walk_finish = False
        self.count_step = 0
        self.stop = False

        self.ik = LegIK() 
        leg_data = self.ik.get_leg_length()
        self.leg_length = leg_data[0] + leg_data[1] + leg_data[2]
        self.joint_position_pub = {}
        self.present_joint_state = {}
        self.all_joint_position_pub = {}

        for joint_name in self.joint_id:
            if self.gazebo_sim:
                self.all_joint_position_pub[joint_name] = rospy.Publisher("/" + joint_name + "_controller/command", Float64, queue_size=1)
        
        for joint_name in self.joint_index:
            self.present_joint_state[joint_name] = JointPosition()
            if self.gazebo_sim:
                self.joint_position_pub[joint_name] = rospy.Publisher("/" + joint_name + "_controller/command", Float64, queue_size=1)
            else:
                self.present_joint_state[joint_name].present_position = self.init_pose_data[joint_name]
       
        if self.gazebo_sim:
            self.walking_param = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param_sim.yaml')
        else:
            self.walking_param = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')

        self.init_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              math.radians(0.0), math.radians(-0.0)]
        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])

        if self.gazebo_sim:
            rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        else:
            rospy.Subscriber('/head_pan_controller/command', HeadState, self.head_pan_controller_callback, queue_size=5)
            rospy.Subscriber('/head_tilt_controller/command', HeadState, self.head_tilt_controller_callback, queue_size=5)
        rospy.Service('/walking/init_pose', Empty, self.init_pose_callback)
        rospy.Service('/walking/command', SetWalkingCommand, self.walking_command_callback)
        rospy.Subscriber('/walking/set_param', WalkingParam, self.set_walking_param_callback, queue_size=5)
        rospy.Subscriber('/app/set_walking_param', AppWalkingParam, self.set_app_walking_param_callback, queue_size=1)
        rospy.Subscriber('/app/set_action', String, self.set_action_callback, queue_size=1)
        rospy.Service('/walking/get_param', GetWalkingParam, self.get_walking_param_callback)
        rospy.Service('/walking/is_walking', GetWalkingState, self.is_walking_callback)
        self.walk_state_pub = rospy.Publisher('/walking/is_walking', Bool, queue_size=1)
        
        self.last_position = None
        self.err = 1e-8
        self.stop_stamp = self.walking_param['trajectory_step_s']
        self.servo_control_cycle = self.walking_param['servo_control_cycle']
        self.servo_max_move = 0
        if self.gazebo_sim:
            self.max_stop_move = 0.5
            self.speed = 0.2/math.radians(360)
        else:
            self.max_stop_move = 0.24
            self.speed = 0.2/math.radians(60)  # 舵机最大速度0.2s/60deg(the maximum speed of servo is 0.2s/60deg)
        time.sleep(0.2)
        rospy.loginfo('ainex controller init finish')
        self.run()

    def run(self): 
        next_time = time.monotonic()
        while not rospy.is_shutdown():
            if self.init_pose_finish: 
                if self.walking_enable:
                    joint_max_move, times_stamp, joint_state = self.walking_module.run(self.present_joint_state)
                    if joint_max_move >= self.max_stop_move:
                        if self.stop:
                            self.walk_state_pub.publish(True)
                        self.stop = False
                        self.servo_max_move = 0
                        data = []
                        for joint_name in self.present_joint_state:
                            if self.walking_param['arm_swing_gain'] != 0 or (self.walking_param['arm_swing_gain'] == 0 and joint_name != 'r_sho_pitch' and joint_name != 'l_sho_pitch'):
                                goal_position = joint_state[joint_name].goal_position
                                if not self.gazebo_sim:
                                    id_ = self.joint_id[joint_name]
                                    pulse = self.angle2pulse(id_, goal_position)
                                    data.extend([[id_, pulse]])
                                    self.present_joint_state[joint_name].present_position = goal_position
                                else:
                                    self.joint_position_pub[joint_name].publish(goal_position)
                                if self.last_position is not None:
                                    d = abs(self.last_position[joint_name].goal_position - goal_position)
                                    if self.servo_max_move < d:
                                        self.servo_max_move = d
                        if not self.gazebo_sim:
                            self.motion_manager.set_servos_position(0, data)
                            # time.sleep(0.013)
                            # self.motion_manager.set_servos_position(int(self.servo_control_cycle*1000), data)
                        curr_time = time.monotonic()
                        delta_sec = curr_time - next_time                        
                        delay_time = self.servo_max_move*self.speed - delta_sec
                        if delay_time > 0:
                            if self.servo_max_move*self.speed < self.servo_control_cycle:
                                time.sleep(self.servo_control_cycle - delta_sec)
                            else:
                                time.sleep(delay_time)
                        else:
                            if delta_sec < self.servo_control_cycle:
                                time.sleep(self.servo_control_cycle - delta_sec)
                        next_time = time.monotonic()
                        self.last_position = copy.deepcopy(joint_state)
                    elif self.walking_module.walk_finish():
                        if not self.stop:
                            self.walk_state_pub.publish(False)
                        self.servo_max_move = 0
                        time.sleep(0.001)
                        self.stop = True
                    else:
                        self.servo_max_move = 0
                        time.sleep(0.001)
                    if times_stamp >= self.walking_param['period_time']/1000.0 - self.walking_param['trajectory_step_s']:
                        if self.walking_param['period_times'] != 0:
                            self.count_step += 1
                            if self.walking_param['period_times'] == self.count_step:
                                self.count_step = 0
                                self.walking_param['period_times'] = 0
                                self.walking_module.stop()
                        else:
                            self.count_step = 0
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.001)

    def init_pose_callback(self, msg):
        self.walking_module.stop()
        while not self.stop:
            time.sleep(0.01)
        self.walking_enable = False
        self.move_to_init_pose()
        self.walking_enable = True
        rospy.loginfo('init pose')
        return EmptyResponse()

    def save_servo_data(self, data, file_name):
        with open("/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/src/ainex_sdk/" + file_name, "w") as file:
            file.write(data + "\n")

    def move_to_init_pose(self):
        # 读取当前舵机角度和记录的角度进行比较，判断是否需要初始化姿态(read and compare the current servo angles and the recorded angles to determine whether to initialize posture)
        self.init_pose_finish = False
        data = self.motion_manager.get_servos_position(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)
        
        read_error = False
        for i in range(len(data)):
            if data[i][0] == 11:
                if data[i][1] < 385:
                    read_error = True
                    self.save_servo_data('11:' + str(i[1]), 'data3')
                if data[i][1] > 570:
                    read_error = True
                    self.save_servo_data('11:' + str(i[1]), 'data3')
            if data[i][0] == 12:
                if data[i][1] < 430:
                    read_error = True
                    self.save_servo_data('12:' + str(i[1]), 'data4')
                if data[i][1] > 615:
                    read_error = True
                    self.save_servo_data('12:' + str(i[1]), 'data4')
        if read_error:
            data = self.motion_manager.get_servos_position(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14)
            for i in range(len(data)):
                if data[i][0] == 11:
                    if data[i][1] < 385:
                        data[i][1] = 500
                    if data[i][1] > 570:
                        data[i][1] = 500
                if data[i][0] == 12:
                    if data[i][1] < 430:
                        data[i][1] = 500
                    if data[i][1] > 615:
                        data[i][1] = 500

        d = 0
        for i in data:
            if self.joint_name[i[0]] in self.joint_index:
                d += abs(self.init_pose_data[self.joint_name[i[0]]] - self.pulse2angle(i[0], i[1]))
        if d > self.max_stop_move:
            for i in data:
                if self.joint_name[i[0]] in self.joint_index:
                    self.present_joint_state[self.joint_name[i[0]]] = JointPosition()
                    self.present_joint_state[self.joint_name[i[0]]].present_position = self.pulse2angle(i[0], i[1])

            self.walking_param = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')       
            self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])
        self.init_servo_data = []
        for joint_name in self.init_pose_data:
            if joint_name not in self.joint_index and joint_name != 'head_pan' and joint_name != 'head_tilt':
                id_ = self.joint_id[joint_name]
                angle = self.init_pose_data[joint_name]
                pulse = self.angle2pulse(id_, angle)
                self.init_servo_data.extend([[id_, pulse]])
        self.motion_manager.set_servos_position(500, self.init_servo_data)
        time.sleep(0.5)
        self.init_pose_finish = True

    def angle2pulse(self, id_, angle):
        # 弧度转舵机脉宽(convert the radian to servo pulse width)
        return self.joint_angles_convert_coef[id_][0] + int(round(angle * self.joint_angles_convert_coef[id_][1]))

    def pulse2angle(self, id_, angle):
        # 舵机脉宽转弧度(convert the servo pulse width to radian)
        return (angle - self.joint_angles_convert_coef[id_][0]) / self.joint_angles_convert_coef[id_][1]

    def joint_states_callback(self, msg):
        # 关节回调，仿真时用到(joint callback for use in simulation)
        if not self.init_pose_finish:
            self.init_pose_finish = True

        for i in range(len(msg.name)):
            if msg.name[i] in self.present_joint_state:
                self.present_joint_state[msg.name[i]].present_position = msg.position[i]

    def head_pan_controller_callback(self, msg):
        # 头部左右控制(control head left and right rotation)
        self.motion_manager.set_servos_position(int(msg.duration*1000), [[self.joint_id['head_pan'], self.angle2pulse(self.joint_id['head_pan'], msg.position)]])

    def head_tilt_controller_callback(self, msg):
        # 头部上下控制(control head to move up and down)
        self.motion_manager.set_servos_position(int(msg.duration*1000), [[self.joint_id['head_tilt'], self.angle2pulse(self.joint_id['head_tilt'], msg.position)]])

    def is_walking_callback(self, msg):
        # 当前是否在移动(determine whether the robot is moving)
        return [not self.stop, "is_walking"]

    def walking_command_callback(self, msg):
        if self.init_pose_finish:
            # 使能控制(enable control)
            if msg.command == "start":
                self.walking_enable = True
                self.walking_module.start()
                self.walking_finish = False
            elif msg.command == "stop":
                self.walking_module.stop()
                while not self.stop:
                    time.sleep(0.01)
            elif msg.command == 'enable':
                self.walking_enable = True
            elif msg.command == 'disable':
                self.walking_module.stop()
                while not self.stop:
                    time.sleep(0.01)
                self.walking_enable = False

        if msg.command == 'enable_control':
            self.init_pose_finish = True
        elif msg.command == 'disable_control':
            self.init_pose_finish = False

        return True

    def set_action_callback(self, msg):
        # 动作组调用服务(action group calling service)
        self.walking_module.stop()
        while not self.stop:
            time.sleep(0.01)
        self.walking_enable = False
        self.init_pose_finish = False
        self.motion_manager.run_action(msg.data)
        self.move_to_init_pose()
        self.walking_enable = True

    def set_app_walking_param_callback(self, msg):
        # APP 设置步态参数,其中一些值固定(set gait parameters with APP, and some values are fixed)
        self.walking_param['period_times'] = 0
        self.walking_param['init_x_offset'] = 0
        self.walking_param['init_y_offset'] = 0
        self.walking_param['init_z_offset'] = msg.height
        if self.body_height_range[0] > self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[0]
        elif self.body_height_range[1] < self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[1]

        self.walking_param['x_move_amplitude'] = msg.x
        if self.x_amplitude_range[0] > self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[0]
        elif self.x_amplitude_range[1] < self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[1]
        self.walking_param['y_move_amplitude'] = msg.y
        if self.y_amplitude_range[0] > self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[0]
        elif self.y_amplitude_range[1] < self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[1]

        self.walking_param['angle_move_amplitude'] = msg.angle
        if self.angle_amplitude_range[0] > self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[0]
        elif self.angle_amplitude_range[1] < self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[1]

        self.walking_param['init_roll_offset'] = 0
        self.walking_param['init_pitch_offset'] = 0
        self.walking_param['init_yaw_offset'] = 0
        self.walking_param['hip_pitch_offset'] = 15
        self.walking_param['z_move_amplitude'] = 0.02
        self.walking_param['pelvis_offset'] = 5
        self.walking_param['move_aim_on'] = False
        self.walking_param['arm_swing_gain'] = 0.5

        if msg.speed == 4:
            self.walking_param['period_time'] = 300
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['init_y_offset'] = -0.008
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['pelvis_offset'] = 5
            self.walking_param['z_move_amplitude'] = 0.015

            # 步幅重新设定
            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(8, self.walking_param['angle_move_amplitude'])
            
            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['init_roll_offset'] = -3
                if self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                    self.walking_param['x_move_amplitude'] = math.copysign(0.012, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['init_roll_offset'] = -3
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])

        elif msg.speed == 3:
            self.walking_param['period_time'] = 400
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['init_y_offset'] = -0.005
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006

            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(8, self.walking_param['angle_move_amplitude'])

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['y_swap_amplitude'] = 0.022
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])
            elif self.walking_param['angle_move_amplitude'] != 0:
                if self.walking_param['y_move_amplitude'] != 0:
                    self.walking_param['init_roll_offset'] = 3
                elif self.walking_param['x_move_amplitude'] != 0:
                    self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])                   

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0
                self.walking_param['init_roll_offset'] = 3
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

            if self.walking_param['x_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.013, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

        elif msg.speed == 2:
            self.walking_param['period_time'] = 500
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['init_y_offset'] = -0.008

            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])
        
            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['init_roll_offset'] = 0
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.022
            elif self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['y_swap_amplitude'] = 0.022

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.028
                self.walking_param['init_roll_offset'] = 3
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0
                self.walking_param['y_move_amplitude'] = math.copysign(0.015, self.walking_param['y_move_amplitude'])
            elif self.walking_param['y_move_amplitude'] != 0 and self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3

            if self.walking_param['x_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.015, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

        elif msg.speed == 1:
            self.walking_param['period_time'] = 600
            self.walking_param['dsp_ratio'] = 0.2
            self.walking_param['step_fb_ratio'] = 0.028
            self.walking_param['y_swap_amplitude'] = 0.02
            self.walking_param['z_swap_amplitude'] = 0.006
            self.walking_param['init_y_offset'] = -0.008
            
            if self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['x_move_amplitude'] = math.copysign(0.01, self.walking_param['x_move_amplitude'])
            if self.walking_param['y_move_amplitude'] != 0:
                self.walking_param['y_move_amplitude'] = math.copysign(0.01, self.walking_param['y_move_amplitude'])
            if self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['angle_move_amplitude'] = math.copysign(10, self.walking_param['angle_move_amplitude'])

            if self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['x_move_amplitude'] == 0:
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3
                self.walking_param['init_y_offset'] = 0
            elif self.walking_param['angle_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] != 0:
                self.walking_param['y_swap_amplitude'] = 0.025
                self.walking_param['init_roll_offset'] = 3
                self.walking_param['init_y_offset'] = 0

            if self.walking_param['y_move_amplitude'] != 0 and self.walking_param['x_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0
                self.walking_param['y_swap_amplitude'] = 0.03
                self.walking_param['init_roll_offset'] = 5
                self.walking_param['y_move_amplitude'] = math.copysign(0.012, self.walking_param['y_move_amplitude'])
                if self.walking_param['y_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0
            elif self.walking_param['y_move_amplitude'] != 0 and self.walking_param['angle_move_amplitude'] != 0:
                self.walking_param['init_y_offset'] = -0.005
                self.walking_param['y_swap_amplitude'] = 0.03
                self.walking_param['init_roll_offset'] = 3

            if self.walking_param['x_move_amplitude'] != 0 and self.walking_param['y_move_amplitude'] == 0 and self.walking_param['angle_move_amplitude'] == 0:
                self.walking_param['init_y_offset'] = 0
                self.walking_param['init_roll_offset'] = 3
                self.walking_param['x_move_amplitude'] = math.copysign(0.015, self.walking_param['x_move_amplitude'])
                if self.walking_param['x_move_amplitude'] > 0:
                    self.walking_param['angle_move_amplitude'] += 0
                else:
                    self.walking_param['angle_move_amplitude'] += 0

        self.walking_module.set_walking_param(self.walking_param)

    def set_walking_param_callback(self, msg):
        # 设置步态参数(set gait parameters)
        self.walking_param['period_times'] = msg.period_times
        self.walking_param['init_x_offset'] = msg.init_x_offset
        self.walking_param['init_y_offset'] = msg.init_y_offset
        self.walking_param['init_z_offset'] = msg.init_z_offset
        if self.body_height_range[0] > self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[0]
        elif self.body_height_range[1] < self.walking_param['init_z_offset']:
            self.walking_param['init_z_offset'] = self.body_height_range[1]
        self.walking_param['init_roll_offset'] = msg.init_roll_offset
        self.walking_param['init_pitch_offset'] = msg.init_pitch_offset
        self.walking_param['init_yaw_offset'] = msg.init_yaw_offset
        self.walking_param['hip_pitch_offset'] = msg.hip_pitch_offset
        self.walking_param['period_time'] = msg.period_time
        self.walking_param['dsp_ratio'] = msg.dsp_ratio
        self.walking_param['step_fb_ratio'] = msg.step_fb_ratio
        self.walking_param['x_move_amplitude'] = msg.x_move_amplitude
        if self.x_amplitude_range[0] > self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[0]
        elif self.x_amplitude_range[1] < self.walking_param['x_move_amplitude']:
            self.walking_param['x_move_amplitude'] = self.x_amplitude_range[1]
        self.walking_param['y_move_amplitude'] = msg.y_move_amplitude
        if self.y_amplitude_range[0] > self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[0]
        elif self.y_amplitude_range[1] < self.walking_param['y_move_amplitude']:
            self.walking_param['y_move_amplitude'] = self.y_amplitude_range[1]
        self.walking_param['z_move_amplitude'] = msg.z_move_amplitude
        if self.step_height_range[0] > self.walking_param['z_move_amplitude']:
            self.walking_param['z_move_amplitude'] = self.step_height_range[0]
        elif self.step_height_range[1] < self.walking_param['z_move_amplitude']:
            self.walking_param['z_move_amplitude'] = self.step_height_range[1]
        self.walking_param['angle_move_amplitude'] = msg.angle_move_amplitude
        if self.angle_amplitude_range[0] > self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[0]
        elif self.angle_amplitude_range[1] < self.walking_param['angle_move_amplitude']:
            self.walking_param['angle_move_amplitude'] = self.angle_amplitude_range[1]
        self.walking_param['y_swap_amplitude'] = msg.y_swap_amplitude
        if self.y_swap_range[0] > self.walking_param['y_swap_amplitude']:
            self.walking_param['y_swap_amplitude'] = self.y_swap_range[0]
        elif self.y_swap_range[1] < self.walking_param['y_swap_amplitude']:
            self.walking_param['y_swap_amplitude'] = self.y_swap_range[1]
        self.walking_param['z_swap_amplitude'] = msg.z_swap_amplitude
        self.walking_param['pelvis_offset'] = msg.pelvis_offset
        self.walking_param['move_aim_on'] = msg.move_aim_on
        self.walking_param['arm_swing_gain'] = msg.arm_swing_gain
        if self.arm_swap_range[0] > self.walking_param['arm_swing_gain']:
            self.walking_param['arm_swing_gain'] = self.arm_swap_range[0]
        elif self.arm_swap_range[1] < self.walking_param['arm_swing_gain']:
            self.walking_param['arm_swing_gain'] = self.arm_swap_range[1] 
        
        self.walking_module.set_walking_param(self.walking_param)

    def get_walking_param_callback(self, msg):
        # 获取步态参数(obtain gait parameters)
        self.walking_param = self.walking_module.get_walking_param()
        
        param = WalkingParam()
        param.init_x_offset = self.walking_param['init_x_offset']
        param.init_y_offset = self.walking_param['init_y_offset']
        param.init_z_offset = self.walking_param['init_z_offset']
        param.init_roll_offset = self.walking_param['init_roll_offset']
        param.init_pitch_offset = self.walking_param['init_pitch_offset']
        param.init_yaw_offset = self.walking_param['init_yaw_offset']
        param.hip_pitch_offset = self.walking_param['hip_pitch_offset']
        param.period_time = self.walking_param['period_time']
        param.dsp_ratio = self.walking_param['dsp_ratio']
        param.step_fb_ratio = self.walking_param['step_fb_ratio']
        param.x_move_amplitude = self.walking_param['x_move_amplitude']
        param.y_move_amplitude = self.walking_param['y_move_amplitude']
        param.z_move_amplitude = self.walking_param['z_move_amplitude']
        param.angle_move_amplitude = self.walking_param['angle_move_amplitude']
        param.y_swap_amplitude = self.walking_param['y_swap_amplitude']
        param.z_swap_amplitude = self.walking_param['z_swap_amplitude']
        param.pelvis_offset = self.walking_param['pelvis_offset']
        param.move_aim_on = self.walking_param['move_aim_on']
        param.arm_swing_gain = self.walking_param['arm_swing_gain']
        param.period_times = 0
        
        return param 

if __name__ == '__main__':
    Controller('ainex_controller')
