#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 巡线跨栏(line following & hurdling)
import time
import rospy
import signal
from ainex_sdk import misc, common
from ainex_example.color_common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ROI

class HurdlesVisualPatrol(Common):
    line_roi = [(5 / 12, 6 / 12, 1 / 4, 3 / 4),
                (6 / 12, 7 / 12, 1 / 4, 3 / 4),
                (7 / 12, 8 / 12, 1 / 4, 3 / 4)
                ]

    hurdles_action_name = 'hurdles'

    hurdles_roi = [1 / 5, 1, 0, 1]

    image_process_size = [160, 120]

    hurdles_x_stop = 395/480
    hurdles_y_stop = 0.5
    hurdles_yaw_stop = 0

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.count = 0
        self.objects_info = []

        self.current_state = "visual_patrol"
        self.next_state = "hurdles"

        self.state = {'visual_patrol': [[500, 260], ['black', self.line_roi, self.image_process_size, self.set_visual_patrol_color], False],
                      'hurdles': [[500, 260], ['blue', self.hurdles_roi, self.image_process_size, self.set_hurdles_color], False]}
        self.head_pan_init = self.state[self.current_state][0][0]  # 左右舵机的初始值(initial value of the left-right servo)
        self.head_tilt_init = self.state[self.current_state][0][1]  # 上下舵机的初始值(initial value of the up-down servo)
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.visual_patrol = VisualPatrol(self.gait_manager)
        self.visual_patrol.update_go_gait(dsp=[300, 0.2, 0.02], x_max=0.01)
        self.visual_patrol.update_turn_gait(dsp=[400, 0.2, 0.02], x_max=0.01) 
        self.approach_object = ApproachObject(self.gait_manager)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果(subscribe to color recognition result)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色(set color)
        self.motion_manager.run_action('walk_ready')

        if rospy.get_param('~start', True):
            self.enter_func(None)
            self.start_srv_callback(None)  # 开启颜色识别(start color recognition)
            common.loginfo('start hurdles')

    def shutdown(self, signum, frame):
        self.running = False

        common.loginfo('%s shutdown' % self.name)

    def set_visual_patrol_color(self, color, roi, image_process_size):
        # 设置巡线颜色
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

    def set_hurdles_color(self, color, roi, image_process_size):
        # 设置台阶标志颜色(set step flag color)
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
        hurdles_param = self.set_stairs_color(msg.data)
        line_param = self.set_visual_patrol_color('black')

        self.detect_pub.publish([line_param, hurdles_param])
        common.loginfo('%s set_color' % self.name)

        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果(obtain color recognition result)
        self.objects_info = msg.data

    def state_init(self, current_state, next_state):
        # 不同阶段的初始化(the initialization in different phrase)
        if self.state[current_state][2] == False:
            self.state[current_state][2] = True
            self.init_action(self.state[current_state][0][0], self.state[current_state][0][1])  # 头部姿态(head posture)
            param1 = self.state[current_state][1][3](self.state[current_state][1][0], self.state[current_state][1][1], self.state[current_state][1][2])
            param2 = self.state[next_state][1][3](self.state[next_state][1][0], self.state[next_state][1][1], self.state[next_state][1][2])
            self.detect_pub.publish([param1, param2])  # 颜色检测设置(color detection settings)

    def exit_hurdles(self, hurdles_data):
        if hurdles_data is not None:
            if self.approach_object.process(max(hurdles_data.y, hurdles_data.left_point[1], hurdles_data.right_point[1]), hurdles_data.x, hurdles_data.angle, 
                                   self.hurdles_x_stop*hurdles_data.height, self.hurdles_y_stop*hurdles_data.width, self.hurdles_yaw_stop, hurdles_data.width, hurdles_data.height):
                self.gait_manager.disable()
                common.loginfo('hurdles')
                self.motion_manager.run_action(self.hurdles_action_name)
                return True
        return False

    def enter_hurdles(self, hurdles_data):
        if hurdles_data is not None:
            if max(hurdles_data.y, hurdles_data.left_point[1], hurdles_data.right_point[1]) > hurdles_data.height / 3:
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    common.loginfo('hurdles approach')
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
                self.state_init(self.current_state, self.next_state)

                line_data = None
                hurdles_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'side':
                        hurdles_data = object_info

                if self.current_state == 'visual_patrol':
                    if line_data is not None:
                        self.visual_patrol.process(line_data.x, line_data.width)
                elif self.current_state == 'hurdles':
                    if self.exit_hurdles(hurdles_data):
                        self.current_state = 'visual_patrol'
                        self.next_state = 'hurdles'
                    else:
                        time.sleep(0.8)

                if self.next_state == 'hurdles':
                    if self.enter_hurdles(hurdles_data):
                        self.current_state = 'hurdles'
                        self.next_state = 'visual_patrol'

                time.sleep(0.01)
            else:
                time.sleep(0.01)
        self.init_action(self.head_pan_init, self.head_tilt_init)
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HurdlesVisualPatrol('hurdles_visual_patrol').run()
