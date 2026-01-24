#!/usr/bin/python3
# coding=utf8
import time
import gpiod
from ainex_sdk.gpio import *

chip = gpiod.chip("gpiochip4")

key = chip.get_line(adapter_board_key)
config = gpiod.line_request()
config.consumer = "key"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
key.request(config)

def get_button_status():
    return key.get_value()

if __name__ == '__main__':
    try:
        while True:
            print('\rkey: {}'.format(key.get_value()), end='', flush=True)  # 打印key状态(print the status of button)
            time.sleep(0.001)
        chip.close()
    except:
        print('按键默认被app_node占用，需要先关闭服务')
        print('~/.stop_ros.sh')
