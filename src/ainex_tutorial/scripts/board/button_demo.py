#!/usr/bin/python3
# coding=utf8
from ainex_sdk import button

while True:
    try:
        print('\rkey: {} '.format(button.get_button_status()), end='', flush=True)  # 打印key状态(print button state)
    except KeyboardInterrupt:
        break
