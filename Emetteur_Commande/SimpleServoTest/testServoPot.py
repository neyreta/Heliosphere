#!/usr/bin/python
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import time

servo_pin = "P8_13"
 
sensor_pin = 'P9_33'
 
ADC.setup()
 
duty_min = 85
duty_max = 95
duty_span = duty_max - duty_min
PWM.start(servo_pin, (100-duty_min), 60.0)
 
while True:
 
        reading = ADC.read(sensor_pin)
        angle = reading          

        anglefl = float(angle)
        duty = 100 - ((anglefl/1.8) * duty_span + duty_min)
        duty = duty*2-15
        print(duty)
        PWM.set_duty_cycle(servo_pin, duty)   
###