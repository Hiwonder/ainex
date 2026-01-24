#!/usr/bin/python3
# coding=utf8
import time
from ainex_sdk import Board

board = Board()
board.set_rgb([[1, 0, 0, 0]])
while True:
    try:
        board.set_rgb([[1, 255, 0, 0]])  # 红(red)
        time.sleep(0.3)
        board.set_rgb([[1, 0, 255, 0]])  # 绿(green)
        time.sleep(0.3)
        board.set_rgb([[1, 0, 0, 255]])  # 蓝(blue)
        time.sleep(0.3)
    except KeyboardInterrupt:
        board.set_rgb([[1, 0, 0, 0]])
        break
