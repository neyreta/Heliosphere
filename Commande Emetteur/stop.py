import time

import pigpio

import lib_para_360_servo
#import lib_standard_servo

#define GPIO for each servo to write to
gpio = 17

pi = pigpio.pi()

para_servo = lib_para_360_servo.write_pwm(pi = pi, gpio = gpio)
#stand_servo = lib_standard_servo.write_wm(pi=pi, gpio = gpio)

#buffer time for initializing everything
time.sleep(1)

para_servo.set_speed(0)
#stand_servo.set_position(-45)

#http://abyz.me.uk/rpi/pigpio/python.html#stop
pi.stop()