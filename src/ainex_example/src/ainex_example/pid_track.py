#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/01
# @author:aiden
# PID追踪(PID tracking)

class PIDTrack:
    def __init__(self, pid, value_range, value=0):
        self.value = value
        self.pid = pid
        self.value_range = value_range

    def update_position(self, value):
        self.value = value

    def update_pid(self, pid):
        self.pid = pid

    def clear(self):
        self.pid.clear()

    def track(self, current, target):
        self.pid.SetPoint = target 
        self.pid.update(current)
        self.value += self.pid.output
        
        if self.value < self.value_range[0]:
            self.value = self.value_range[0]
        if self.value > self.value_range[1]:
            self.value = self.value_range[1]

        return self.value
