import time

import pigpio

import lib_para_360_servo

#define GPIO for each servo to read from
gpio_d_r = 16

#define GPIO for each servo to write to
gpio_d_w = 17

pi = pigpio.pi()

#### Calibrate para_360_servo, speed  = 0.2 and -0.2

para_360_servo = lib_para_360_servo.write_pwm(pi = pi, gpio = gpio_d_w)

#buffer time for initializing everything
time.sleep(1)
para_360_servo.set_speed(0.2)
servo = lib_para_360_servo.calibrate_pwm(pi = pi, gpio = gpio_d_r)
para_360_servo.set_speed(0)

#http://abyz.me.uk/rpi/pigpio/python.html#stop
pi.stop()