'''!
@file servo_driver.py
This file contains code that creates a class to control a servo from the STM Nucleo-64 using MicroPython. This file is 
imported to the board for integration with other files as a class, but also includes test code at the bottom which can be run
from a PC. The class initializes a arduino object from a timer, timer channel, and pin inputs. The class inlcudes 1 method which
is used to set the servo angle. As written, the set_pos() angle calculations are hardware specific and must be changed based on
specific servo characterisitcs. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date January March 19th, 2024
'''


import micropython
import time
import pyb


class ServoDriver:
    """! 
    This class implements a servo driver for an ME405 kit. The class contains two functions: init and set_pos. 
    The functions will be explained in further detail below. 
    """
    def __init__ (self, servo_pin, timer, timer_channel ):
        """! 
        Creates a motor driver by initializing GPIO
        pins and timer. 

        @param servo_pin This is the value for the CPU pin needed to control the servo, input as a string. 
        @param timer This is the timer used with the choses GPIO pin, input as an integer. 
        @param timer_channel This is the timer channel on chosen timer which corresponds to the GPIO pin, input as an integer.  
        """
       

        servo_pin = getattr(pyb.Pin.board, servo_pin) # get the pin value for the pin store in en_pin
        
    
        self.servo_pin=pyb.Pin(servo_pin, pyb.Pin.OUT_PP, value=0)   #initialize the pin as output pin
        t = pyb.Timer(timer, freq=1000) #start the timer 
        t.prescaler(80) #set prescaler to get clock frquency to 1 Mhz, not 80
        self.timer_channel=t.channel(timer_channel,pyb.Timer.PWM, pin=self.servo_pin)  #start PWM on the timer channel for the pin


                 
    def set_pos(self,angle):
        """!
        This method sets the servo output angle by 
        controlling PWM of the GPIO pin. 
        @param angle The desired setpoint angle in degrees.  
        """
        PWM_angle=(angle/270)*2000+500     #converts an angle in degrees to the pwm needed for the servo controller. 500 microseconds 
                                            # is 0 degrees, 2500 is 180 degrees
        PWM_angle = int(PWM_angle)
        self.timer_channel.pulse_width(PWM_angle)

if __name__ == "__main__":  # test code contained below
    servo1 = ServoDriver('PA5',2,1)
    servo1.set_pos(160)
    time.sleep(1)
    servo1.set_pos(195)
    time.sleep(1)
    servo1.set_pos(160)
    
    