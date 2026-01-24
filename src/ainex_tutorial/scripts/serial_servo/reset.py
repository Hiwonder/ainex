#!/usr/bin/env python3
# encoding: utf-8
# 将所有舵机的范围设为0-1000(set all servos to range 0 to 1000)
from ainex_sdk import Board

servo_control = Board()
for i in range(11, 15):
    print(i)
    servo_control.bus_servo_set_angle_limit(i, 0, 1000)        
