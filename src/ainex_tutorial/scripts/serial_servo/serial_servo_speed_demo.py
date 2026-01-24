#!/usr/bin/env python3
# encoding: utf-8
# 改变舵机的转动速度(change the rotation speed of the servo)
import time
from ainex_sdk import Board

servo_control = Board()
print('id23 serial servo move at different speed')

while True:
    try:
        servo_id = 23  # 舵机id(0-253)(servo ID (0-253))
        position = 400  # 位置(0-1000)(position (0-1000))
        duration = 0.5  # 时间(0.02-30)(time (0.02-30))
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
        
        position = 600
        duration = 0.5
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)

        position = 400  # 位置(0-1000)(position (0-1000))
        duration = 1  # 时间(0.02-30)(time (0.02-30))
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
        
        position = 600
        duration = 1
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        time.sleep(duration)
    except KeyboardInterrupt:
        position = 500
        duration = 0.5
        servo_control.bus_servo_set_position(duration, [[servo_id, position]])
        break
