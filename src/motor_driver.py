'''!
@file motor_driver.py
This file contains code that creates a class to control a motor for ME-405 Lab01. The class was then imported to the board for testing as motor_driver.py.
The motor is first initilzied, setting up the pins needed to control the motor, and then can the duty cycle and direction of the motor can be controlled. 
Test code is included in the bottom of this file that will not run when this file is imported to the board. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date January 28th, 2024
'''
import micropython
import time
import pyb
# have to import pyb for it work on the board

class MotorDriver:
    """! 
    This class implements a motor driver for an ME405 kit. The class contains two functions: init and set_duty cycle. 
    The functions will be explained in further detail below. 
    """
    def __init__ (self, en_pin, in1pin, in2pin, timer):
        """! 
        Creates a motor driver by initializing GPIO
        pins and turning off the motor for safety. 

        @param en_pin This is the value for the CPU pin needed to control the motor. This value is input as a string of the pin name.
                The pin is set to high in the code to enable motor control.
        @param in1pin This is the value for the first pin name needed to control the motor. This value is input as a string of the pin name.
        @param in2pin This is the value for the second pin name needed to control the motor. This value is input as a string of the pin name.
        @param timer This is the value for the timer channel of the motor. Set as a integer. 
        """
        en_pin = getattr(pyb.Pin.board, en_pin) # get the pin value for the pin store in en_pin
        in1pin = getattr(pyb.Pin.board, in1pin)
        in2pin = getattr(pyb.Pin.board, in2pin)
 
        self.ENx = pyb.Pin (en_pin, pyb.Pin.OUT_PP, value = 0)  # init the CPU pin
        self.IN1x = pyb.Pin (in1pin, pyb.Pin.OUT_PP, value = 0) # init the first pin
        self.IN2x = pyb.Pin (in2pin, pyb.Pin.OUT_PP, value = 0) # init the second pin
        self.t = pyb.Timer(timer, freq=1000)    # start the timer from the timer value
        self.tch1 = self.t.channel(1,pyb.Timer.PWM, pin=self.IN1x)  # create the timer channels for each motor pin
        self.tch2 = self.t.channel(2,pyb.Timer.PWM, pin=self.IN2x)
        self.ENx.high() # set the motor pin to high 
        print ("Creating a motor driver")

    def set_duty_cycle (self, level):
        """!
        This method sets the duty cycle to be sent
        to the motor to the given level. Positive values
        cause torque in one direction, negative values
        in the opposite direction.
        @param level A signed integer holding the duty
               cycle of the voltage sent to the motor 
        """
        if level >= 0:  # test the sign of motor
            self.tch1.pulse_width_percent(0)    # if positive turn the motor one direction based on the value of level
            self.tch2.pulse_width_percent(level)
        else:
            self.tch2.pulse_width_percent(0)    # turn the motor the other way if negative
            self.tch1.pulse_width_percent(-1*level) # set level as an absolute value
                
        print (f"Setting duty cycle to {level}")


if __name__ == "__main__":  # test code contained below

    #motor1 = MotorDriver ('PC1', 'PA0', 'PA1', 5)  # set the necessary pin names as string
    motor1=MotorDriver('PC1','PA0','PA1',5)
    motor2=MotorDriver('PA10','PB4','PB5',3)
    #encoder1=EncoderReader('PC6','PC7',8)
    
    # code below to cycle from values -100 to 100
    #perc = -100
    
    while True:
#         try:
#             for i in range(200):
#                     perc = perc + 1
#                     motor1.set_duty_cycle(perc) 
#                     time.sleep(0.05)
#             for i in range(200):
#                     perc = perc - 1
#                     motor1.set_duty_cycle(perc)  
#                     time.sleep(0.05)
        try:
            motor1.set_duty_cycle(50)
            time.sleep(0.05)
            
        except KeyboardInterrupt:
            
            motor1.set_duty_cycle(0)
            break

   

    
    
   
   
   
   
