#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/06
import time
import math
import rospy
from std_msgs.msg import Bool
from ainex_interfaces.msg import WalkingParam
from ainex_interfaces.srv import GetWalkingParam, SetWalkingCommand

class GaitManager:
    def __init__(self):
        self.state = 'enable'
        self.is_walking = False
        
        self.err = 1e-8

        # 以下参数范围是测试的比较好的范围，如果需要修改请缓慢增加数值，再进行测试(the following parameter ranges are recommended for testing. If you need to modify them, please increase the values slowly and test again)
        # period_time, dsp_ratio, y_swap_amplitude
        # 步态周期ms, 双脚触地占比(0-1), 摆动幅度m(Gait period in ms, percentage of time when both feet are on the ground (0-1), and swing amplitude in meters)
        self.dsp_ratio = [[300, 0.2, 0.02],
                          [400, 0.2, 0.02],
                          [500, 0.2, 0.02],
                          [600, 0.1, 0.04]
                          ]

        # 躯体可以比完全站直低的范围，单位m(the range of the body height for standing up, in units of m)
        self.body_height_range = [0.015, 0.06] 
        # 步幅范围，单位m(the range of stride, in units of m)
        self.x_amplitude_range = [0.0, 0.02]
        self.y_amplitude_range = [0.0, 0.02]
        # 腿抬高范围，单位m(the range of height for lifting legs, in units of m)
        self.step_height_range = [0.01, 0.04]
        # 转弯范围，单位度(the range of turning, in units of degrees)
        self.rotation_angle_range = [0, 10]
        # 手臂摆动范围，单位度(the range of arm swing, in units of radians)
        self.arm_swap_range = [0, 60]

        self.y_swap_range = [0, 0.05]
        self.dsp_ratio_range = [0, 1]

        rospy.wait_for_service('/walking/get_param')
        res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
        self.walking_param = res.parameters
        self.param_pub = rospy.Publisher('/walking/set_param', WalkingParam, queue_size=1)
        rospy.Subscriber('/walking/is_walking', Bool, self.walking_state_callback, queue_size=1)
        time.sleep(0.2)

    def walking_state_callback(self, msg):
        self.is_walking = msg.data

    def get_gait_param(self):
        '''
        param body_height: 躯体离完全站直的距离(m)， 默认0.015(Distance from the body to fully upright position in meters. Default is 0.015)
        param step_height: 抬腿高度(m)， 默认0.02(Height of leg lift in meters. Default is 0.02)
        param pelvis_offset: 髋关节左右摆动角度(deg)， 默认5(Angle of hip swing in degrees. Default is 5)
        param hip_pitch_offset: 髋关节前后倾斜角度(deg)， 默认20(Angle of hip tilt in degrees. Default is 20)
        param z_swap_amplitude: 躯体上下摆动幅度(m)， 默认0.01(Amplitude of body up and down swing in meters. Default is 0.01)
        '''
        walking_param = {}
        walking_param['init_x_offset'] = self.walking_param.init_x_offset
        walking_param['init_y_offset'] = self.walking_param.init_y_offset
        walking_param['body_height'] = self.walking_param.init_z_offset
        walking_param['init_roll_offset'] = self.walking_param.init_roll_offset
        walking_param['init_pitch_offset'] = self.walking_param.init_pitch_offset
        walking_param['init_yaw_offset'] = self.walking_param.init_yaw_offset
        walking_param['hip_pitch_offset'] = self.walking_param.hip_pitch_offset
        walking_param['step_fb_ratio'] = self.walking_param.step_fb_ratio
        walking_param['step_height'] = self.walking_param.z_move_amplitude
        walking_param['angle_move_amplitude'] = self.walking_param.angle_move_amplitude
        walking_param['z_swap_amplitude'] = self.walking_param.z_swap_amplitude
        walking_param['pelvis_offset'] = self.walking_param.pelvis_offset
        walking_param['move_aim_on'] = self.walking_param.move_aim_on

        return walking_param

    def update_pose(self, walking_param):
        if self.state == 'disable':
            self.state = 'enable'
            rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable')

        if walking_param['body_height'] - self.body_height_range[1] > self.err or walking_param['body_height'] - self.body_height_range[0] < -self.err:
            raise Exception('body_height %d out of range(0.015~0.06)' % walking_param['body_height'])
        if walking_param['step_height'] - self.step_height_range[1] > self.err or walking_param['step_height'] - self.step_height_range[0] < -self.err:
            raise Exception('step_height %d out of range(0.01~0.04)' % walking_param['step_height'])

        self.walking_param.init_x_offset = walking_param['init_x_offset']
        self.walking_param.init_y_offset = walking_param['init_y_offset']
        self.walking_param.init_z_offset = walking_param['body_height']
        self.walking_param.init_roll_offset = walking_param['init_roll_offset']
        self.walking_param.init_pitch_offset = walking_param['init_pitch_offset']
        self.walking_param.init_yaw_offset = walking_param['init_yaw_offset']
        self.walking_param.hip_pitch_offset = walking_param['hip_pitch_offset']
        self.walking_param.step_fb_ratio = walking_param['step_fb_ratio']
        self.walking_param.z_move_amplitude = walking_param['step_height']
        self.walking_param.angle_move_amplitude = walking_param['angle_move_amplitude']
        self.walking_param.z_swap_amplitude = walking_param['z_swap_amplitude']
        self.walking_param.pelvis_offset = walking_param['pelvis_offset']
        self.walking_param.move_aim_on = walking_param['move_aim_on']

        self.walking_param.x_move_amplitude = 0
        self.walking_param.y_move_amplitude = 0
        self.walking_param.angle_move_amplitude = 0

        self.param_pub.publish(self.walking_param)

    def update_param(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param=None, arm_swap=30, step_num=0):
        if step_velocity[0] < 0:
            raise Exception('period_time cannot be negative' % step_velocity[0])
        if step_velocity[1] > self.dsp_ratio_range[1] or step_velocity[1] < self.dsp_ratio_range[0]:
            raise Exception('dsp_ratio_range %d out of range(0~1)' % step_velocity[1])
        if step_velocity[2] - self.y_swap_range[1] > self.err or step_velocity[2] < self.y_swap_range[0]:
            raise Exception('y_swap_range %d out of range(0~0.05)' % step_velocity[2])
        if abs(x_amplitude) - self.x_amplitude_range[1] > self.err or abs(x_amplitude) < self.x_amplitude_range[0]:
            raise Exception('x_amplitude %d out of range(-0.02~0.02)' % x_amplitude)
        if abs(y_amplitude) - self.y_amplitude_range[1] > self.err or abs(y_amplitude) < self.y_amplitude_range[0]:
            raise Exception('y_amplitude %d out of range(-0.02~0.02)' % y_amplitude)
        if abs(rotation_angle) - self.rotation_angle_range[1] > self.err or abs(rotation_angle) < self.rotation_angle_range[0]:
            raise Exception('rotation_angle %d out of range(-10~10)' % rotation_angle)
        if abs(arm_swap) - self.arm_swap_range[1] > self.err or arm_swap < self.arm_swap_range[0]:
            raise Exception('arm_swap %d out of range(0~60)' % arm_swap)
        if step_num < 0:
            raise Exception('step_num cannot be negative' % step_num)

        if walking_param is not None:
            if walking_param['body_height'] - self.body_height_range[1] > self.err or walking_param['body_height'] - self.body_height_range[0] < -self.err:
                raise Exception('body_height %d out of range(0.015~0.06)' % walking_param['body_height'])
            if walking_param['step_height'] - self.step_height_range[1] > self.err or walking_param['step_height'] - self.step_height_range[0] < -self.err:
                raise Exception('step_height %d out of range(0.01~0.04)' % walking_param['step_height'])

            self.walking_param.init_x_offset = walking_param['init_x_offset']
            self.walking_param.init_y_offset = walking_param['init_y_offset']
            self.walking_param.init_z_offset = walking_param['body_height']
            self.walking_param.init_roll_offset = walking_param['init_roll_offset']
            self.walking_param.init_pitch_offset = walking_param['init_pitch_offset']
            self.walking_param.init_yaw_offset = walking_param['init_yaw_offset']
            self.walking_param.hip_pitch_offset = walking_param['hip_pitch_offset']
            self.walking_param.step_fb_ratio = walking_param['step_fb_ratio']
            self.walking_param.z_move_amplitude = walking_param['step_height']
            self.walking_param.angle_move_amplitude = walking_param['angle_move_amplitude']
            self.walking_param.z_swap_amplitude = walking_param['z_swap_amplitude']
            self.walking_param.pelvis_offset = walking_param['pelvis_offset']
            self.walking_param.move_aim_on = walking_param['move_aim_on']

        self.walking_param.period_time = step_velocity[0]
        self.walking_param.dsp_ratio = step_velocity[1]
        self.walking_param.y_swap_amplitude = step_velocity[2]
        self.walking_param.x_move_amplitude = x_amplitude
        self.walking_param.y_move_amplitude = y_amplitude
        self.walking_param.angle_move_amplitude = rotation_angle
        self.walking_param.arm_swing_gain = math.radians(arm_swap)
        self.walking_param.period_times = step_num
        self.param_pub.publish(self.walking_param)

    def set_body_height(self, body_height, use_time):
        if self.state == 'disable':
            self.state = 'enable'
            rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable')
        times = int(abs(body_height - self.walking_param.init_z_offset)/0.005)
        for i in range(times):
            self.walking_param.init_z_offset += math.copysign(0.005, body_height - self.walking_param.init_z_offset)
            self.param_pub.publish(self.walking_param)
            time.sleep(use_time/times)

    def set_step(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param=None, arm_swap=30, step_num=0):
        '''
        以设置参数行走
        param step_velocity: 列表形式包含三个参数(a list containing three parameters)[period_time, dsp_ratio, y_swap_amplitude], 即周期(ms), 占地时间比例， 左右摆动幅度(m)(which are gait period (ms), percentage of time when both feet are on the ground (0-1), and swing amplitude in meters)
        param x_amplitude: x方向步幅(m)(step length in the x direction in meters)
        param y_amplitude: y方向步幅(m)(step length in the y direction in meters)
        param rotation_angle: 旋转幅度(deg)(rotation angle in degrees)
        param walking_param: 其他参数(other gait parameters)
        param arm_swap: 手臂摆动幅度(deg)， 默认30, 当为0时不会给手臂发送指令(amplitude of arm swing in degrees. Default is 30. If it is set to 0, no command will be sent to the arms)
        param step_num: 步数(number of steps to take)
        '''
        try:
            self.update_param(step_velocity, x_amplitude, y_amplitude, rotation_angle, walking_param, arm_swap, step_num)

            if step_num != 0:
                res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
                self.walking_param = res.parameters
                rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')
                while not self.is_walking:
                    time.sleep(0.01)
                while self.is_walking:
                    time.sleep(0.01)
                self.state = 'step_walking'
            else:
                if self.state != 'walking':
                    if step_num == 0:
                        self.state = 'walking'
                        res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
                        self.walking_param = res.parameters
                        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')
        except BaseException as e:
            print(e)
            return

    def move(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, arm_swap=30, step_num=0):
        '''
        param step_velocity: 速度选择分三档分别为 1，2，3, 4速度由快到慢(Speed selection, which can be 1, 2, or 3. 4 is the slowest speed)
        param x_amplitude: x方向步幅(m)(Step length in the x direction in meters)
        param y_amplitude: y方向步幅(m)(Step length in the y direction in meters)
        param rotation_angle: 旋转幅度(deg)(Rotation angle in degrees)
        param arm_swap: 手臂摆动幅度(deg)， 默认30, 当为0时不会给手臂发送指令(Amplitude of arm swing in degrees. Default is 30. If it is set to 0, no command will be sent to the arms)
        '''
        if 0 < step_velocity < 5:
            self.set_step(self.dsp_ratio[step_velocity - 1], x_amplitude, y_amplitude, rotation_angle, self.get_gait_param(), arm_swap, step_num)

    def stop(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('stop')
        self.state ='stop'

    def disable(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('disable')
        self.state = 'disable'

    def enable(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable')
        self.state = 'enable'

if __name__ == '__main__':
    rospy.init_node('walk_test')
    gait_manager = GaitManager()
    # gait_manager.move(1, 0.02, 0, 0)
    # time.sleep(2)
    # gait_manager.stop()
    # gait_manager.move(2, -0.02, 0, 0)
    # time.sleep(2)
    # gait_manager.stop()
    # gait_manager.move(3, 0, 0, 10)
    # time.sleep(2)
    gait_manager.set_step([400, 0.1, 0.03], 0, -0.01, 0, arm_swap=0, step_num=1)
    # gait_manager.stop()
    # gait_manager.move(4, 0.0, 0.01, 0, step_num=0)
    # gait_manager.move(3, 0.02, 0, 0, step_num=3)
    # print('stop')
