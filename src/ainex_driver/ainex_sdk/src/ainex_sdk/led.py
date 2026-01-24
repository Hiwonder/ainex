#!/usr/bin/python3
# coding=utf8
import time
import gpiod

led_pin = 16  # 蓝色led(the blue LED)
chip = gpiod.chip('gpiochip4')

led = chip.get_line(led_pin)
config = gpiod.line_request()
config.consumer="led"
config.request_type=gpiod.line_request.DIRECTION_OUTPUT
led.request(config)

def set_value(value):
    led.set_value(value)

if __name__ == '__main__':
    print('led闪烁: 0.1/s')
    try:
        while True:
            led.set_value(0)
            time.sleep(0.1)
            led.set_value(1)
            time.sleep(0.1)
        chip.close()
    except:
        print('led默认被hw_wifi占用，需要自行注释掉相关代码')
        print('然后重启服务sudo systemctl restart hw_wifi.service')
