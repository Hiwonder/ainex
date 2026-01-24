#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/10
# @author:aiden
# 上阶梯or下阶梯(up and down steps)
import time
import rospy
import signal
from ainex_sdk import common
from ainex_example.color_common import Common
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class ClimbStairsNode(Common):
    # 阶梯的roi(ROI of the steps)
    # y_min, y_max, x_min, x_max分别表示占图像的比例(y_min, y_max, x_min, and x_max represent respectively the proportions of the image)
    stairs_roi = [1/5, 1, 0, 1]

    # 上下阶梯动作(action of up and down steps)
    climb_stairs_action_name = 'climb_stairs'       # 上台阶(up steps)
    descend_stairs_action_name = 'descend_stairs'   # 下台阶(down steps)

    # 图像处理时缩放到这个分辨率(When process the image, scale it to this resolution)
    image_process_size = [160, 120]

    # 上阶梯x方向的停止条件(stop conditions for up steps in the x direction)
    # x距离目标位置小于图片宽度的385/480时停止(stop when the x distance to the target position is less than 385/480 of the image width)
    climb_stairs_x_stop = 385/480  
    # 上阶梯y方向的停止条件(stop conditions for up steps in the y direction)
    # y距离目标位置小于图片高度的一半时停止(stop when the y distance to the target position is less than half of the image height)
    climb_stairs_y_stop = 0.5
    # 上阶梯航向角停止条件(stop conditions for up steps in the yaw direction)
    # 允许一定的航向角误差(allow for a certain amount of yaw error)
    climb_stairs_yaw_stop = 0


    # 下阶梯x方向的停止条件(stop conditions for down steps in the x direction)
    # x距离目标位置小于图片宽度的385/480时停止(stop when the x distance to the target position is less than 385/480 of the image width)
    descend_stairs_x_stop = 385/480
    # 下阶梯y方向的停止条件(stop conditions for down steps in the y direction)
    # y距离目标位置小于图片高度的一半时停止(stop when the y distance to the target position is less than half of the image height)
    descend_stairs_y_stop = 0.5
    # 上阶梯航向角停止条件(stop conditions for down steps in the yaw direction)
    # 允许一定的航向角误差(allow for a certain amount of yaw error)
    descend_stairs_yaw_stop = 0 

    def __init__(self, name):
        rospy.init_node(name)       # 初始化ROS节点(initialize ROS node)
        self.name = name
        self.running = True
        self.objects_info = []
        self.current_state = 'climb_stairs' # 当前状态(current state)

        self.state = {'climb_stairs':   [[500, 260], ['red', self.stairs_roi, self.image_process_size, self.set_stairs_color], False],
                      'descend_stairs': [[500, 260], ['red', self.stairs_roi, self.image_process_size, self.set_stairs_color], False]}
        # 初始化头部姿态(initialize the head posture)
        self.head_pan_init = self.state[self.current_state][0][0]   # 左右舵机的初始值(initial value of the left-right servo)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial value of the up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        # 初始化相关节点和模块(initialize related nodes and modules)
        self.approach_object = ApproachObject(self.gait_manager)

        # 更新逼近停止的判断条件(update the judgment conditions for approaching and stopping)
        # x方向允许的距离误差,像素为单位(distance error allowed in the x direction, in units of pixels)
        # y方向允许的距离误差,像素为单位(distance error allowed in the y direction, in units of pixels)
        # 航向角允许的误差,度为单位(error allowed in the yaw angle, in units of degrees)
        self.approach_object.update_approach_stop_value(40, 50, 6)
        # 第一个参数40表示目标位置x方向的容差范围,单位是像素。也就是机器人x方向距离目标位置在50像素之内就认为逼近完成。(The first parameter 40 represents the tolerance range in the x direction of the target position, in units of pixels. That is, the robot is considered to have approached the target position when it is within 50 pixels in the x direction)
        # 第二个参数50表示目标位置y方向的容差范围。同理,在60像素之内认为y方向逼近完成。(The second parameter 50 represents the tolerance range in the y direction of the target position. Similarly, it is considered to have approached the target position when it is within 60 pixels in the y direction)
        # 第三个参数6表示目标位置的航向角容差范围,单位是度。在7度角度误差之内就认为航向角逼近完成。(The third parameter 6 represents the tolerance range of the target position's yaw angle, in degrees. It is considered to have approached the target position when the yaw angle error is within 7 degrees)

        self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(move the hand back to prevent obstruction)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
 
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            self.enter_func(None)
            self.start_srv_callback(None)  # 开启颜色识别(start color recognition)
            common.loginfo('start climb_stairs')

    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    def set_stairs_color(self, color, roi, image_process_size):
        # 设置台阶标志颜色(set the color of the step flag)
        stairs_param = ColorDetect()
        stairs_param.color_name = color
        stairs_param.detect_type = 'side'
        stairs_param.use_name = True
        stairs_param.image_process_size = image_process_size
        stairs_param.roi.y_min = int(roi[0] * image_process_size[1])
        stairs_param.roi.y_max = int(roi[1] * image_process_size[1])
        stairs_param.roi.x_min = int(roi[2] * image_process_size[0])
        stairs_param.roi.x_max = int(roi[3] * image_process_size[0])
        stairs_param.min_area = 10 * 20
        stairs_param.max_area = image_process_size[0] * image_process_size[1]

        return stairs_param

    def set_color_srv_callback(self, msg):
        # 设置颜色(set color)
        stairs_param = self.set_stairs_color(msg.data)
        self.detect_pub.publish([stairs_param])
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data

    def climb_stairs_process(self, stairs_data):
        # 上阶梯处理(up steps processing)
        if self.approach_object.process(max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]), stairs_data.x, stairs_data.angle, 
                                        self.climb_stairs_x_stop*stairs_data.height, self.climb_stairs_y_stop*stairs_data.width, self.climb_stairs_yaw_stop, stairs_data.width, stairs_data.height):
            self.gait_manager.disable()  # 关闭步态控制(close gait control)
            common.loginfo('climb_stairs')
            self.motion_manager.run_action(self.climb_stairs_action_name)  # 执行上台阶动作(execute up-steps action)
            return True
        else:
            return False

    def descend_stairs_process(self, stairs_data):
        # 下阶梯处理(down steps processing)
        if self.approach_object.process(max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]), stairs_data.x, stairs_data.angle, 
                                        self.descend_stairs_x_stop*stairs_data.height, self.descend_stairs_y_stop*stairs_data.width, self.descend_stairs_yaw_stop, stairs_data.width, stairs_data.height):
            self.gait_manager.disable()  # 关闭步态控制(close gait control)
            common.loginfo('descend_stairs')
            self.motion_manager.run_action(self.descend_stairs_action_name)  # 执行下台阶动作(execute down-steps action)
            return True
        else:
            return False

    def run(self):
        while self.running:
            if self.start:
                # 设置颜色识别参数(set color recognition parameter)
                if self.state[self.current_state][2] == False:
                    self.state[self.current_state][2] = True
                    self.init_action(self.state[self.current_state][0][0], self.state[self.current_state][0][1])  # 头部姿态(head posture)
                    param = self.state[self.current_state][1][3](self.state[self.current_state][1][0],
                                                                 self.state[self.current_state][1][1],
                                                                 self.state[self.current_state][1][2])
                    self.detect_pub.publish([param])  # 颜色检测设置(color detection settings)

                # 获取识别结果(obtain recognition result)
                stairs_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'side':
                        stairs_data = object_info
                # 处理对应的上下阶梯逻辑(process corresponding up and down steps logic)
                if stairs_data is not None and self.current_state == 'climb_stairs':
                    if self.climb_stairs_process(stairs_data):
                        self.running = False
                    else:
                        time.sleep(0.8)

                if stairs_data is not None and self.current_state == 'descend_stairs':
                    if self.descend_stairs_process(stairs_data):
                        self.running = False
                    else:
                        time.sleep(0.8)
            else:
                time.sleep(0.01)
        # 退出处理(exit processing)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ClimbStairsNode('climb_stairs').run()
