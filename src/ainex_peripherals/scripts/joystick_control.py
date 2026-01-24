#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy
from ainex_kinematics.gait_manager import GaitManager

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState():
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)
        self.board = Board()
        self.period_time = [400, 0.2, 0.02]
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.time_stamp_ry = 0 
        self.angle_move_amplitude = 0
        self.init_z_offset = 0.025
        self.count_stop = 0 
        self.status = 'stop'
        self.update_height = False
        self.update_param = False
        self.last_axes = dict(zip(AXES_MAP, [0.0,] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0,] * len(BUTTON_MAP)))
        self.mode = 0

        time.sleep(0.2)
        
        # 机器人步态库调用
        self.gait_manager = GaitManager()
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

    def axes_callback(self, axes):
        self.x_move_amplitude = 0.00
        self.angle_move_amplitude = 0.00
        self.y_move_amplitude = 0.00
        self.period_time = [400, 0.2, 0.02]
        if axes['ly'] > 0.3:
            self.update_param = True
            self.x_move_amplitude = 0.013
        elif axes['ly'] < -0.3:
            self.update_param = True
            self.x_move_amplitude = -0.013
        if axes['lx'] > 0.3:
            self.period_time[2] = 0.025
            self.update_param = True
            self.y_move_amplitude = 0.015
        elif axes['lx'] < -0.3:
            self.period_time[2] = 0.025
            self.update_param = True
            self.y_move_amplitude = -0.015

        if axes['rx'] > 0.3:
            self.update_param = True
            self.angle_move_amplitude = 8
        elif axes['rx'] < -0.3:
            self.update_param = True
            self.angle_move_amplitude = -8
         
        if self.update_param:
            self.gait_param = self.gait_manager.get_gait_param()
            self.gait_param['init_z_offset'] = self.init_z_offset
            self.gait_manager.set_step(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
            # time.sleep(0.06)
        if self.status == 'stop' and self.update_param:
            self.status = 'move'
        elif self.status == 'move' and not self.update_param:
            self.status = 'stop'
            self.gait_manager.stop()
        self.update_param = False

    def callback(self, axes):
        if rospy.get_time() > self.time_stamp_ry:
            self.update_height = False
            if axes['ry'] < -0.5:
                self.update_height = True
                self.init_z_offset += 0.005
                if self.init_z_offset > 0.06:
                    self.update_height = False
                    self.init_z_offset = 0.06
            elif axes['ry'] > 0.5:
                self.update_height = True
                self.init_z_offset += -0.005
                if self.init_z_offset < 0.025:
                    self.update_height = False
                    self.init_z_offset = 0.025
            if self.update_height and not self.update_param:
                self.gait_param = self.gait_manager.get_gait_param()
                self.gait_param['body_height'] = self.init_z_offset
                self.gait_manager.update_param(self.period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, self.gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass 

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass 

    def triangle_callback(self, new_state):
        pass 

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            self.gait_param = self.gait_manager.get_gait_param()
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                for i in range(t):
                    self.init_z_offset += 0.005 * abs(0.025 - self.init_z_offset) / (0.025 - self.init_z_offset)
                    self.gait_param['body_height'] = self.init_z_offset
                    self.gait_manager.update_param(self.period_time, 0.0, 0.0, 0.0, self.gait_param, step_num=1)
                    time.sleep(0.05)

    def hat_xl_callback(self, new_state):
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))
        self.callback(axes)
        for key, value in axes.items(): # 轴的值被改变(the value of the axis is changed)
            if key != 'ry':
                if self.last_axes[key] != value:
                    axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                rospy.logerr(str(e))
        for key, value in buttons.items():
            new_state = ButtonState.Normal
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                # rospy.loginfo(key + ': ' + str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        rospy.logerr(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    node = JoystickController()
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

