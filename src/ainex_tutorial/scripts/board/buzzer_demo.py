#!/usr/bin/python3
# coding=utf8
import time
from ainex_sdk import Board

board = Board()
print('buzzer di at 1hz')

try:
    while True:
        board.set_buzzer(1900, 0.1, 0.9, 1)
        time.sleep(1)
        board.set_buzzer(2400, 0.1, 0.4, 0)
        time.sleep(2)
except KeyboardInterrupt:
    board.set_buzzer(1900, 0, 0, 1)  # buzzer off
