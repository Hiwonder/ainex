#!/usr/bin/env python3
# encoding: utf-8
import time
import serial

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200(initialize the serial port with the baud rate of 115200)
print('serial read data at 100hz')

while True:
    print(serialHandle.read())
    time.sleep(0.01)
