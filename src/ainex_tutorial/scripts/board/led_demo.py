#!/usr/bin/python3
# coding=utf8
import time
from ainex_sdk import led

print('led blink at 5hz')

try:
    while True:
        led.set_value(1)  # led亮(LED on)
        time.sleep(0.1)  # 延时(delay)
        led.set_value(0)  # led灭(LED off)
        time.sleep(0.1)
except KeyboardInterrupt:
    led.set_value(0)  # 关闭程序时将led熄灭(turn off the LED when closing the program)

