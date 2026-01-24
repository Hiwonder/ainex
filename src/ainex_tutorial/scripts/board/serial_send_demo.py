#!/usr/bin/env python3
# encoding: utf-8
import time
import serial

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  # 初始化串口， 波特率为115200(initialize the serial port with the baud rate of 115200)

print('serial send "hello" at 10hz')

while True:
    data = b"hello\r\n"
    serialHandle.write(data)
    time.sleep(0.1)
