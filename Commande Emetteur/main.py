import pigpio

import time

import lib_motion

pi = pigpio.pi()

tower = lib_motion.control(pi = pi)

tower.set_speed(0.2)
time.sleep(5)
tower.set_speed(0.5)
time.sleep(5)
tower.set_speed(0)
time.sleep(10)
a = 0
while a < 4:
    tower.move_para(45)
    a+=1

tower.move_para(20)
tower.move_para(-20)

a = 0
while a < 2:
    tower.move_para(-90)
    a+=1

#http://abyz.me.uk/rpi/pigpio/python.html#callback
tower.cancel()

#http://abyz.me.uk/rpi/pigpio/python.html#stop
pi.stop()