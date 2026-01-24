#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/13
# @author:aiden
# 巡线+上下阶梯+跨栏x2(line following & up and down steps & hurdling x2)
import time
import rospy
import signal
from ainex_sdk import misc, common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ROI

class CombinationNode(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出(check three ROIs in order, and immediately exit if a black line is detected)
    # y_min, y_max, x_min, x_max分别表示占图像的比例, 即实际大小为y_min*height(y_min, y_max, x_min, and x_max represent respectively the proportion of the image occupied. The actual size is y_min*height)
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]

    stairs_roi = [1 / 5, 1, 0, 1]
    hurdles_roi = [1 / 5, 1, 0, 1]

    # 所需动作的名称(the name of required action)
    hurdles_action_name = 'hurdles'
    climb_stairs_action_name = 'climb_stairs'
    descend_stairs_action_name = 'descend_stairs'

    # 图像处理时缩放到这个分辨率， 不建议修改(When process the image, scale it to this resolution. Do not suggest to modify)
    image_process_size = [160, 120]

    enter_climb_stairs_y = 280 / 480  # 当检测到的标识像素坐标y值占图像的比例大于此值时进入此阶段(When the detected object's y-coordinate in the image is greater than this value, it enters this phrase)
    climb_stairs_x_stop = 0.5  # 当检测到的标识像素坐标x值占图像的比例在此值附近(范围可在ApproachObject里设置)时停止前后移动(When the detected object's x-coordinate in the image is within a certain range (specified in the ApproachObject function) based on the proportion of the image, the robot will stop moving forward or backward)
    climb_stairs_y_stop = 375 / 480  # 当检测到的标识像素坐标y值占图像的比例在此值附近(范围可在ApproachObject里设置)时停止横向移动(When the detected object's y-coordinate in the image is within a certain range (specified in the ApproachObject function) based on the proportion of the image, the robot will stop moving left or right)
    climb_stairs_yaw_stop = 0  # 当检测到的标识角度在此值附近(范围可在ApproachObject里设置)时停止旋转移动(When the detected object's orientation angle is within a certain range (specified in the ApproachObject function), the robot will stop rotating)

    enter_descend_stairs_y = 280/480
    descend_stairs_x_stop = 0.5
    descend_stairs_y_stop = 390 / 480
    descend_stairs_yaw_stop = 0

    enter_hurdles_y = 340 / 480
    hurdles_x_stop = 0.5
    hurdles_y_stop = 410/480
    hurdles_yaw_stop = 0

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.count = 0
        self.running = True
        self.objects_info = []

        self.current_state = "visual_patrol"
        self.next_state = "climb_stairs"

        self.state = {'visual_patrol':  [[500, 260], ['black', self.line_roi, self.image_process_size, self.set_visual_patrol_color], False],
                      'climb_stairs':   [[500, 260], ['red', self.stairs_roi, self.image_process_size, self.set_stairs_color], False],
                      'descend_stairs': [[500, 260], ['red', self.stairs_roi, self.image_process_size, self.set_stairs_color], False],
                      'hurdles1': [[500, 260], ['blue', self.hurdles_roi, self.image_process_size, self.set_hurdles_color], False],
                      'hurdles2': [[500, 260], ['blue', self.hurdles_roi, self.image_process_size, self.set_hurdles_color], False]}
        
        self.head_pan_init = self.state[self.current_state][0][0]   # 左右舵机的初始值(initial values of left and right servos)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial value of up and down servos)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.approach_object = ApproachObject(self.gait_manager)
        self.visual_patrol = VisualPatrol(self.gait_manager)
        # 减小巡线步幅，提高其他标志检测稳定性(Decrease the stride of line following, improving the stability of other flag detection)
        self.visual_patrol.update_go_gait(dsp=[300, 0.2, 0.02], x_max=0.01)
        self.visual_patrol.update_turn_gait(dsp=[400, 0.2, 0.02], x_max=0.01)

        signal.signal(signal.SIGINT, self.shutdown)
        
        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        self.motion_manager.run_action('walk_ready')

        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画(Notify the color recognition to prepare, at this time only display the camera original image)
            self.enter_func(None)
            self.start_srv_callback(None)
            common.loginfo('start ombination')

    def shutdown(self, signum, frame):
        self.running = False
        common.loginfo('%s shutdown' % self.name)

    def set_visual_patrol_color(self, color, roi, image_process_size):
        # 设置巡线颜色(set the color of line to be followed)
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
        stairs_param.min_area = 10*20
        stairs_param.max_area = image_process_size[0]*image_process_size[1]
        
        return stairs_param

    def set_hurdles_color(self, color, roi, image_process_size):
        # 设置台阶标志颜色(set the color of the step flag)
        hurdles_param = ColorDetect()
        hurdles_param.color_name = color
        hurdles_param.detect_type = 'side'
        hurdles_param.use_name = True
        hurdles_param.image_process_size = image_process_size
        hurdles_param.roi.y_min = int(roi[0] * image_process_size[1])
        hurdles_param.roi.y_max = int(roi[1] * image_process_size[1])
        hurdles_param.roi.x_min = int(roi[2] * image_process_size[0])
        hurdles_param.roi.x_max = int(roi[3] * image_process_size[0])
        hurdles_param.min_area = 10 * 20
        hurdles_param.max_area = image_process_size[0] * image_process_size[1]

        return hurdles_param

    def set_color_srv_callback(self, msg):
        # 设置颜色(set color)
        stairs_param = self.set_stairs_color(self.state['climb_stairs'][1][0])
        line_param = self.set_visual_patrol_color(self.state['visual_patrol'][1][0])
        hurdles_param = self.set_stairs_color(self.state['hurdles'][1][0])

        self.detect_pub.publish([line_param, stairs_param, hurdles_param])
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data

    def state_init(self, current_state, next_state):
        # 不同阶段的初始化(initialization in different phrases)
        if self.state[current_state][2] == False:
            self.state[current_state][2] = True
            self.init_action(self.state[current_state][0][0], self.state[current_state][0][1])  # 头部姿态(head posture)
            param1 = self.state[current_state][1][3](self.state[current_state][1][0], self.state[current_state][1][1], self.state[current_state][1][2])
            param2 = self.state[next_state][1][3](self.state[next_state][1][0], self.state[next_state][1][1], self.state[next_state][1][2])
            self.detect_pub.publish([param1, param2])  # 颜色检测设置(color detection settings)
            common.loginfo(current_state + ' init')

    def exit_climb_stairs(self, stairs_data):
        # 上阶梯处理(up steps processing)
        if stairs_data is not None:
            if self.approach_object.process(max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]), stairs_data.x, stairs_data.angle, 
                                        self.climb_stairs_y_stop*stairs_data.height, self.climb_stairs_x_stop*stairs_data.width, self.climb_stairs_yaw_stop, stairs_data.width, stairs_data.height):
                self.gait_manager.disable()  # 关闭步态控制(close gait control)
                common.loginfo('climb_stairs')
                self.motion_manager.run_action(self.climb_stairs_action_name)  # 执行上台阶动作(execute up-steps action)
                return True
        return False

    def exit_descend_stairs(self, stairs_data):
        # 下阶梯处理(down steps processing)
        if stairs_data is not None:
            if self.approach_object.process(max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]), stairs_data.x, stairs_data.angle, 
                                        self.descend_stairs_y_stop*stairs_data.height, self.descend_stairs_x_stop*stairs_data.width, self.descend_stairs_yaw_stop, stairs_data.width, stairs_data.height):
                self.gait_manager.disable()  # 关闭步态控制(close gait control)
                common.loginfo('descend_stairs')
                self.motion_manager.run_action(self.descend_stairs_action_name)  # 执行下台阶动作(execute down-steps action)
                return True
        return False

    def exit_hurdles(self, hurdles_data):
        # 上阶梯处理(up steps processing)
        if hurdles_data is not None:
            if self.approach_object.process(max(hurdles_data.y, hurdles_data.left_point[1], hurdles_data.right_point[1]), hurdles_data.x, hurdles_data.angle, 
                                   self.hurdles_y_stop*hurdles_data.height, self.hurdles_x_stop*hurdles_data.width, self.hurdles_yaw_stop, hurdles_data.width, hurdles_data.height):
                self.gait_manager.disable()
                common.loginfo('hurdles')
                self.motion_manager.run_action(self.hurdles_action_name)
                return True
        return False

    def enter_climb_stairs(self, stairs_data):
        if stairs_data is not None:
            if max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]) > self.enter_climb_stairs_y * stairs_data.height:
                self.count += 1
                if self.count > 5:  # 主线程比较快，颜色检测回调慢一点，需要连续检测来排除滞后干扰(If the main thread is faster than the color detection callback, and continuous detection is needed to eliminate lag interference)
                    self.count = 0
                    self.gait_manager.disable()
                    self.approach_object.update_approach_stop_value(20, 30, 6)  # 设置靠近目标停止的条件，分别为y, x, angle误差(Set the conditions for stopping when approaching to the object, which are y, x, and angle error respectively)
                    self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(move the hand back to prevent obstruction)
                    time.sleep(0.5)
                    return True
            else:
                self.count = 0
        return False

    def enter_descend_stairs(self, stairs_data):
        if stairs_data is not None:
            if max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]) > self.enter_descend_stairs_y * stairs_data.height:
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    self.gait_manager.disable()
                    self.approach_object.update_approach_stop_value(20, 30, 6)
                    self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(move the hand back to prevent obstruction)
                    time.sleep(0.5)
                    return True
            else:
                self.count = 0
        return False

    def enter_hurdles(self, hurdles_data):
        if hurdles_data is not None:
            if max(hurdles_data.y, hurdles_data.left_point[1], hurdles_data.right_point[1]) > self.enter_hurdles_y * hurdles_data.height:
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    self.gait_manager.disable()
                    self.approach_object.update_approach_stop_value(x_approach_value=30, yaw_approach_value=5)
                    self.motion_manager.run_action('hand_back')  # 手往后，防止遮挡(move the hand back to prevent obstruction)
                    return True
            else:
                self.count = 0
        return False

    def run(self):
        while self.running:
            if self.start:
                # 获取识别结果
                line_data = None
                side_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'side':
                        side_data = object_info

                # 当前阶段处理完成，回到巡线(The current processing is completed, and return to line following)
                if self.current_state == 'visual_patrol':
                    if line_data is not None:
                        self.visual_patrol.process(line_data.x, line_data.width)
                elif self.current_state == 'climb_stairs':
                    if self.exit_climb_stairs(side_data):
                        self.current_state = 'visual_patrol'
                        self.next_state = 'descend_stairs'
                        self.state[self.current_state][2] = False  # 重新初始化当前阶段(initialize current phrase again)
                        common.loginfo('exit climb_stairs ---> enter visual_patrol')
                    else:
                        time.sleep(0.8)
                elif self.current_state == 'descend_stairs':
                    if self.exit_descend_stairs(side_data):
                        self.current_state = 'visual_patrol'
                        self.next_state = 'hurdles1'
                        self.state[self.current_state][2] = False
                        common.loginfo('exit descend_stairs ---> enter visual_patrol')
                    else:
                        time.sleep(0.8)
                elif self.current_state == 'hurdles1':
                    if self.exit_hurdles(side_data):
                        self.current_state = 'visual_patrol'
                        self.next_state = 'hurdles2'
                        self.state[self.current_state][2] = False
                        common.loginfo('exit hurdles1 ---> enter visual_patrol')
                    else:
                        time.sleep(0.8)
                elif self.current_state == 'hurdles2':
                    if self.exit_hurdles(side_data):
                        self.current_state = 'visual_patrol'
                        self.next_state = 'climb_stairs'
                        self.state[self.current_state][2] = False
                        common.loginfo('exit hurdles2 ---> enter visual_patrol')
                    else:
                        time.sleep(0.8)

                # 是否退出巡线，进入下一阶段(whether to exit the line following to enter the next phrase)
                if self.next_state == 'climb_stairs':
                    if self.enter_climb_stairs(side_data):
                        self.current_state = 'climb_stairs'
                        self.next_state = 'visual_patrol'
                        common.loginfo('exit visual_patrol ---> enter climb_stairs')
                elif self.next_state == 'descend_stairs':
                    if self.enter_descend_stairs(side_data):
                        self.current_state = 'descend_stairs'
                        self.next_state = 'visual_patrol'
                        common.loginfo('exit visual_patrol ---> enter descend_stairs')
                elif self.next_state == 'hurdles1':
                    if self.enter_hurdles(side_data):
                        self.current_state = 'hurdles1'
                        self.next_state = 'visual_patrol'
                        common.loginfo('exit visual_patrol ---> enter hurdles1')
                elif self.next_state == 'hurdles2':
                    if self.enter_hurdles(side_data):
                        self.current_state = 'hurdles2'
                        self.next_state = 'visual_patrol'
                        common.loginfo('exit visual_patrol ---> enter hurdles2')

                self.state_init(self.current_state, self.next_state)

                time.sleep(0.01)  # 防止空载(prevent the robot from running without load)
            else:
                time.sleep(0.01)
        
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    CombinationNode('combination').run()
