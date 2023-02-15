import collections
import time

import pigpio

#https://www.parallax.com/sites/default/files/downloads/900-00005-Standard-Servo-Product-Documentation-v2.2.pdf

class write_pwm:
    """
    Stears a servo, in this case a Parallax Standard Servo stand_data_sheet_ .

    This class stears a Parallax Standard Servo and should also work with other servos which 
    have a 50Hz PWM for setting the position. The position of the Parallax Standard Servo 
    can be set between -90 (``min_degree``) and +90 (``max_degree``) degree.

    .. note::
        ``min_pw`` and ``max_pw`` might needed to be interchanged, depending on if ``min_pw`` is 
        moving the servo to max_right/clockwise or max_left/counter-clockwise,
        see methods :meth:`max_left` and :meth:`max_right`. 
        :meth:`max_right` -> ``min_pw`` should let the servo rotate clockwise.

    .. warning::
        Be carefull with setting the min and max pulsewidth! Test carefully ``min_pw`` and ``max_pw``
        before setting them. Wrong values can damage the servo, see set_servo_pulsewidth_ !!!

    :param pigpio.pi pi: 
        Instance of a pigpio.pi() object.
    :param int gpio:
        GPIO identified by their Broadcom number, see elinux.org_ .
        To this GPIO the signal wire of the servo has to be connected.
    :param int min_pw:
        Min pulsewidth, see **Warning**, carefully test the value before!
        **Default:** 1000, taken from set_servo_pulsewidth_ .
    :param int max_pw:
        Max pulsewidth, see **Warning**, carefully test the value before!
        **Default:** 2000, taken from set_servo_pulsewidth_ .
    :param int min_degree:
        Min degree which the servo is able to move.
        **Default:** -90, taken from stand_data_sheet_ .
    :param int max_degree:
        Max degree which the servo is able to move.
        **Default:** +90, taken from stand_data_sheet_ .       

    .. _elinux.org: https://elinux.org/RPi_Low-level_peripherals#Model_A.2B.2C_B.2B_and_B2
    .. _stand_data_sheet: https://www.parallax.com/sites/default/files/downloads/900-00005-Standard-Servo-Product-Documentation-v2.2.pdf
    .. _set_servo_pulsewidth: http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
    """

    def __init__(self, pi, gpio, min_pw = 1000, max_pw = 2000, min_degree = -90, max_degree = 90):

        self.pi = pi
        self.gpio = gpio
        self.min_pw = min_pw
        self.max_pw = max_pw
        self.min_degree = min_degree
        self.max_degree = max_degree
        #calculate slope for calculating the pulse width
        self.slope = (self.min_pw - ((self.min_pw + self.max_pw)/2)) / self.max_degree
        #calculate y-offset for calculating the pulse width
        self.offset = (self.min_pw + self.max_pw)/2

    def set_pw(self, pulse_width):
        """
        Sets pulsewidth of the PWM.

        This method allows setting the pulsewidth of the PWM directly. This can be used to
        test which ``min_pw`` and ``max_pw`` are appropriate. For this the ``min_pw`` and 
        ``max_pw`` are needed to be set very small and very big, so that they do not limit 
        the set pulsewidth. Normally they are used to protect the servo by limiting 
        the pulsewidth to a certain range.

        .. warning::
            Be carefull with setting the min and max pulsewidth! Test carefully ``min_pw`` and ``max_pw``
            before setting them. Wrong values can damage the servo, see set_servo_pulsewidth_ !!!

        :param int,float pulsewidth:
            Pulsewidth of the PWM signal. Will be limited to ``min_pw`` and ``max_pw``.

        .. _set_servo_pulsewidth: http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
        """
        
        #http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth
        pulse_width = max(min(self.max_pw, pulse_width), self.min_degree)

        self.pi.set_servo_pulsewidth(user_gpio = self.gpio, pulsewidth = pulse_width)
        
    def calc_pw(self, degree):

        pulse_width = self.slope * degree + self.offset
        
        return pulse_width

    def set_position(self, degree):
        """
        Sets position of the servo in degree.

        This method sets the servo position in degree. Minus is to the left, 
        plus to the right.

        :param int,float degree:
            Should be between ``min_degree`` (max left) and ``max_degree`` 
            (max right), otherwise the value will be limited to those values.
        """
        
        degree = max(min(self.max_degree, degree), self.min_degree)

        calculated_pw = self.calc_pw(degree = degree)
        self.set_pw(pulse_width = calculated_pw)

    def middle_position(self):
        """
        Sets the position of the servo to 0 degree, so middle position.
        """

        pulse_width = (self.min_pw+self.max_pw)/2
        self.set_pw(pulse_width = pulse_width)

    def max_left(self):
        """
        Sets the position of the servo to -90 degree, so ``min_degree`` (max left, 
        counter-clockwise).
        """
        
        self.set_pw(self.max_pw)

    def max_right(self):
        """
        Sets the position of the servo to 90 degree, so ``max_degree`` (max right, 
        clockwise).
        """

        self.set_pw(self.min_pw)

if __name__ == '__main__':

    #just continue
    pass