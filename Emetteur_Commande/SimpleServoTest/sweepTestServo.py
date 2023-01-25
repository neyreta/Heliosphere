
import Adafruit_BBIO.PWM as PWM

PWM.start("P8_13", 95.0, 60, 1)
PWM.set_duty_cycle("P8_13", 97.0)
PWM.stop("P8_13")
PWM.cleanup()