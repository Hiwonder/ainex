#!/usr/bin/python3
# coding=utf8
import time
from ainex_sdk import Board

board = Board()
board.enable_reception()

while True:
    res = board.get_imu()
    if res is not None:
        for item in res:
            print(" {: .8f}".format(item), end='')
        print()
    time.sleep(0.01)
