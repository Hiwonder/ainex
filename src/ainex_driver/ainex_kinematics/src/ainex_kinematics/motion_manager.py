#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/10
# 串口舵机控制库(serial servo control library)
import os
import time
import rospy
import sqlite3 as sql
from ros_robot_controller.srv import GetBusServosPosition
from ros_robot_controller.msg import SetBusServosPosition, BusServoPosition

class MotionManager:
    runningAction = False
    stopRunning = False
    
    def __init__(self, action_path='/home/ubuntu/software/ainex_controller/ActionGroups'):
        self.servo_position = {}
        self.action_path = action_path
        self.get_servo_position_srv = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_position', GetBusServosPosition)
        self.servo_state_pub = rospy.Publisher('/ros_robot_controller/bus_servo/set_position', SetBusServosPosition, queue_size=1)
        time.sleep(0.2)

    def set_servos_position(self, duration, *positions):
        '''
        控制多个舵机转动(control multiple servos rotation)
        :param duration: 时间ms(time ms)
        :param args: 舵机id和位置(servo ID and position), [[1, 500], [2, 500], ...]
        '''
        # t = time.time()
        # board.bus_servo_set_position(duration/1000.0, *positions)
        # print(time.time() - t)
        msg = SetBusServosPosition()
        msg.duration = float(duration/1000.0)
        position_list = []
        for i in positions[0]:
            position = BusServoPosition()
            position.id = i[0]
            position.position = i[1]
            position_list.append(position)
            self.servo_position[str(i[0])] = i[1]
        msg.position = position_list
        self.servo_state_pub.publish(msg)

    def get_servos_position(self, *args, fake=False):
        '''
        获取多个舵机位置(obtain multiple servos position)
        :param args: 舵机id(servo ID)
        '''
        if fake:
            data = []
            for i in self.servo_position:
                data.extend([[int(i), self.servo_position[i]]])
        else: 
            data = []
            result = self.get_servo_position_srv(list(args))
            for i in result.position:
                data.extend([[i.id, i.position]])
        return data 

    def stop_action_group(self):
        self.stopRunning = True

    def run_action(self, actNum):
        '''
        运行动作组，无法发送stop停止信号(Run the specified action group. Cannot send a stop signal to stop the action)
        :param actNum: 动作组名字 ， 字符串类型(the name of the action group as a string)
        :param times:  运行次数(the number of times to run the action group)
        :return:
        '''
        if actNum is None and self.action_path is not None:
            return
        actNum = os.path.join(self.action_path, actNum + ".d6a")
        self.stopRunning = False
        if os.path.exists(actNum):
            if not self.runningAction:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                while True:
                    act = cu.fetchone()
                    if self.stopRunning:
                        self.stopRunning = False                   
                        break
                    if act is not None:
                        data = []
                        for i in range(0, len(act) - 2, 1):
                            data.extend([[i + 1, act[2 + i]]])
                        self.set_servos_position(act[1], data) 
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出(wait for the action group to finish before exiting)
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            print('can not find aciton file')

if __name__ == '__main__':
    motion_manager = MotionManager(action_path='/home/ubuntu/software/ainex_controller/ActionGroups')
    rospy.init_node('test')
    # 单个舵机运行(run a single servo)
    motion_manager.set_servos_position(500, [[23, 300]])
    time.sleep(0.5) 
    
    # 多个舵机运行(run multiple servos)
    motion_manager.set_servos_position(500, [[23, 500], [24, 500]])
    time.sleep(0.5)
    
    # 执行动作组(execute action group)
    motion_manager.run_action('left_shot')
    motion_manager.run_action('right_shot')
