#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/10
import time
import rospy
from ainex_kinematics.gait_manager import GaitManager

rospy.init_node('simple_gait_control_demo')
gait_manager = GaitManager()
time.sleep(0.2)

# 控制机器人行走(Control the robot to walk)
gait_manager.move(1, 0.01, 0, 0)
time.sleep(3)  # 以速度1前进3秒然后停下(Move forward at a speed of 1 for 3 seconds, then stop)
gait_manager.stop()

gait_manager.move(2, -0.01, 0, 0, arm_swap=0)  # 关闭手臂摆动(stop swing the robotic arms)
time.sleep(3)  # 后退3秒然后停下( Move backward for 3 seconds, then stop)
gait_manager.stop()

gait_manager.move(2, 0, 0, 5)
time.sleep(3)  # 右转3秒(Turn right for 3 seconds)

gait_manager.move(3, 0.01, 0, 5)
time.sleep(3)  # 前进同时右转3秒(Move forward and turn right for 3 seconds)

gait_manager.move(2, 0, 0.01, 0, step_num=3)  # 控制行走步数(Control the number of walking steps)

gait_manager.move(2, -0.01, 0, 0)
time.sleep(3)  # 后退3秒然后停下(Move backward for 3 seconds, then stop)
gait_manager.stop()
