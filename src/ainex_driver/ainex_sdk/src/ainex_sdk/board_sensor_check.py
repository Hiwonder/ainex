#!/usr/bin/python3
# coding=utf8
# 测试板载传感器，包括led, rgb, button, buzzer, imu(test onboard sensors, including LED, RGB, button, buzzer, and IMU)
import time
import threading
from ainex_sdk import button, led, Board

button_count = 0
button_status = 'off'
board = Board()
board.enable_reception()

def led_check():
    while True:
        led.set_value(1)
        board.set_rgb([[1, 255, 0, 0]])
        time.sleep(0.2)
        board.set_rgb([[1, 0, 255, 0]])
        time.sleep(0.2)
        led.set_value(0)
        board.set_rgb([[1, 0, 0, 255]])
        time.sleep(0.2)
        board.set_rgb([[1, 0, 0, 0]])
        time.sleep(0.2)

threading.Thread(target=led_check, daemon=True).start()

while True:
    try:
        res = board.get_imu()
        if res is not None:
            for item in res:
                print("{: .8f}".format(item), end='')
            print()
        button_off = button.get_button_status()
        if not button_off and button_status == 'off':
            button_count = 0
            board.set_buzzer(1900, 0.1, 0.01, 1)
            button_status = 'on'
        elif button_off:
            button_count += 1
            if button_count > 5:
                button_status = 'off'
                button_count = 0
        time.sleep(0.01)
    except KeyboardInterrupt:
        led.set_value(0)
        board.set_rgb([[1, 0, 0, 0]])
        break
