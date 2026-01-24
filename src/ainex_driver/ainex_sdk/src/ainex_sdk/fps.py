#!/usr/bin/python3
# coding=utf8
import cv2
import time
from ainex_sdk import common

class FPS:
    def __init__(self):
        self.fps = 0.0
        self.last_time = 0
        self.current_time = 0
        self.confidence = 0.1

    def update(self):
        self.last_time = self.current_time
        self.current_time = time.time()
        new_fps = 1.0 / (self.current_time - self.last_time)
        if self.fps == 0.0:
            self.fps = new_fps if self.last_time != 0 else 0.0
        else:
            self.fps = new_fps * self.confidence + self.fps * (1.0 - self.confidence)
        return float(self.fps)

    def show_fps(self, img):
        fps_text = 'FPS: {:.2f}'.format(self.fps)
        common.putText(img, fps_text)
        return img

