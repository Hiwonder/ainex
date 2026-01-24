#!/usr/bin/env python3
# encoding: utf-8
# 读取舵机的状态(read the state of the servo)
import time
from ainex_sdk import Board

servo_control = Board()
servo_control.enable_reception()
print('get serial servo status')

while True:
    try:
        servo_id = 24
        # servo_id = servo_control.bus_servo_read_id()[0]  # 如果只连接一个舵机且id未知(if only connecting one servo and the ID is unknown)
        pos = servo_control.bus_servo_read_position(servo_id)
        dev = servo_control.bus_servo_read_offset(servo_id)
        angle_range = servo_control.bus_servo_read_angle_limit(servo_id)
        vin_range = servo_control.bus_servo_read_vin_limit(servo_id)
        temperature_warn = servo_control.bus_servo_read_temp(servo_id)
        temperature = servo_control.bus_servo_read_temp_limit(servo_id)
        vin = servo_control.bus_servo_read_vin(servo_id)
        torque_state = servo_control.bus_servo_read_torque_state(servo_id)

        print('id:%s'%(str(servo_id).ljust(3)))
        print('pos:%s'%(str(pos).ljust(4)))
        print('dev:%s'%(str(dev).ljust(4)))
        print('angle_range:%s'%(str(angle_range).ljust(4)))
        print('voltage_range:%smV'%(str(vin_range).ljust(5)))
        print('temperature_warn:%s°C'%(str(temperature_warn)))
        print('temperature:%s°C'%(str(temperature)))
        print('vin:%smV'%(str(vin).ljust(4)))
        print('lock:%s'%(str(torque_state).ljust(4)))
        print('------------------------------')
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
