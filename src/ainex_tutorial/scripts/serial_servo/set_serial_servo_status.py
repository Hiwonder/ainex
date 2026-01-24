#!/usr/bin/env python3
# encoding: utf-8
# 设置舵机的状态(set servo state)
from ainex_sdk import Board

servo_control = Board()
servo_control.enable_reception()

print('只能连接一个舵机进行操作！')
print('set serial servo status')

def get_status():
    servo_id = servo_control.bus_servo_read_id()[0]
    dev = servo_control.bus_servo_read_offset(servo_id)
    angle_range = servo_control.bus_servo_read_angle_limit(servo_id)
    vin_range = servo_control.bus_servo_read_vin_limit(servo_id)
    temperature_warn = servo_control.bus_servo_read_temp_limit(servo_id)
    torque_state = servo_control.bus_servo_read_torque_state(servo_id)
    print('*******current status**********')
    print('id:%s'%(str(servo_id).ljust(3)))
    print('dev:%s'%(str(dev).ljust(4)))
    print('angle_range:%s'%(str(angle_range).ljust(4)))
    print('voltage_range:%s'%(str(vin_range).ljust(5)))
    print('temperature_warn:%s'%(str(temperature_warn).ljust(4)))
    print('lock:%s'%(str(torque_state).ljust(4)))
    print('*******************************')

print('''
--------------------------               
1 id                
2 dev               
3 save_dev          
4 angle_range       
5 voltage_range   
6 temperature_warn
7 lock              
8 help
9 exit
--------------------------''')

while True:
    try:
        get_status()
        mode = int(input('input mode:'))
        if mode == 1:
            oldid = int(input('current id:'))
            newid = int(input('new id(0-253):'))
            servo_control.bus_servo_set_id(oldid, newid)
        elif mode == 2:
            servo_id = int(input('servo id:'))
            dev = int(input('deviation(-125~125):'))
            if dev < 0:
                dev = 0xff + dev + 1
            servo_control.bus_servo_set_offset(servo_id, dev)
        elif mode == 3:
            servo_id = int(input('servo id:'))
            servo_control.bus_servo_save_offset(servo_id)
        elif mode == 4:
            servo_id = int(input('servo id:'))
            min_pos = int(input('min pos(0-1000):'))
            max_pos = int(input('max pos(0-1000):'))
            servo_control.bus_servo_set_angle_limit(servo_id, [min_pos, max_pos])        
        elif mode == 5:
            servo_id = int(input('servo id:'))
            min_vin = int(input('min vin(4500-14000):'))
            max_vin = int(input('max vin(4500-14000):'))
            servo_control.bus_servo_set_vin_limit(servo_id, [min_vin, max_vin])  
        elif mode == 6:
            servo_id = int(input('servo id:'))
            min_temp = int(input('temperature(0-85):'))
            servo_control.bus_servo_set_temp_limit(servo_id, min_temp)  
        elif mode == 7:
            servo_id = int(input('servo id:'))
            status = int(input('status:'))
            servo_control.bus_servo_enable_torque(servo_id, status) 
        elif mode == 8:
            print('''
--------------------------
1 id                
2 dev               
3 save_dev          
4 angle_range       
5 voltage_range   
6 temperature_warn
7 lock              
8 help
9 exit
--------------------------''')
        elif mode == 9:
            break
        else:
            print('error mode')
    except KeyboardInterrupt:
        break
