#!/usr/bin/python
import Adafruit_BBIO.ADC as ADC
import time
ADC.setup()
while True:
    value = ADC.read("P9_33") * 1.8
    print(round(value, 2))
    time.sleep(.5)