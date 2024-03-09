'''!
@file encoder_reader.py 
This file contains code which creates an encoder class for completion of ME-405 Week 2. The class created in this code can be used to set up an encoder that can
read motor posiiton values during its use. Once the encoder is initialized properly, the read function can read the position change of the motor, and the zero function
can reset the position count from the encoder. The class and its functions are described in further detail below. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date February 8th, 2024
'''

import micropython
import time
import pyb  # have to import pyb for it work on the board when imported as a class
# import motor_driver.py    # can use the previous MotorDriver class to test the encoder 


class EncoderReader:
    """! 
    
    """
    def __init__ (self, in1pin, in2pin, timer):
        """! 
        Creates a encoder by initializing GPIO
        pins, setting up the correct timer channels, and creating the values
        needed to calculate the position change of the motor. 

        @param in1pin This is the value for the first pin name needed to set up the encoder. This value is input as a string of the pin name.
        @param in2pin This is the value for the second pin name needed to set up the encoder. This value is input as a string of the pin name.
        @param timer This is the value for the timer channel of the motor. Set as a integer. 
        """
        # get the pin values for the pin store
        in1pin = getattr(pyb.Pin.board, in1pin) # getattr finds the value attached to the given string in in1pin
        in2pin = getattr(pyb.Pin.board, in2pin)
 
        # set up the correct pin values
        self.pin1 = pyb.Pin(in1pin, pyb.Pin.IN) # store the pin variables in the encoder class
        self.pin2 = pyb.Pin(in2pin, pyb.Pin.IN)
        
        # set up the timer
        self.enc_timer = pyb.Timer(timer, freq = 1000)  # the timer value is from the function ran to init the encoder
        
        # set up the two timer channels needed to run the encoder
        self.enc_channel_1 = self.enc_timer.channel(1, pyb.Timer.ENC_AB, pin = self.pin1)   # pyb.Timer.ENC_AB inits the timer channels as encoder channels
        self.enc_channel_2 = self.enc_timer.channel(2, pyb.Timer.ENC_AB, pin = self.pin2)
        
        # init the variables for read position of the motor
        ## delta
        # store the change in position between different read functions of the encoder
        self.delta=0    
        ## positon
        # absolute total position of the motor from the encoder
        # this variable will hold the total counts and will increase or decrease based on the direction of the motor
        self.pos=0	# absolute total position
        ## previous position 
        # this variable holds the previous number from each encoder read
        self.prev_pos=0
        print ("Creating an encoder!")

    def read (self):
        """!
        This function reads the change of position of the motor from the encoder.
        It stores the current encoder value by using enc_timer.counter(), then conducts an algorithim
        to detect whether there has been an under or overflow since the last encoder read. 
        The function then adjust the value in case of an under/overflow to read the correct position change value from the encoder. 
        """

        self.value = self.enc_timer.counter()   # store the current position in self.value
        self.delta=self.value-self.prev_pos # calculate delta
    
        if self.delta<=-(16000+1)/2:    # compare delta to an under or overflow
            self.delta=self.delta+16001 # correctly change delta to achieve the correct encoder value
        elif self.delta>=(16000+1)/2:
            self.delta=self.delta-16001
                     
        self.pos=self.pos+self.delta    # add delta to the absolute total position
        self.prev_pos=self.value    # set value as previous position
    
    def zero(self):
        """!
        This function zeros the values stored needed for encoder calculations to reset the encoder count. 
        The values of delta and position are reset to zero while storing the last value from an encoder read
        in the prev_pos variable allow the next call of the read function to work properly. 
        """
        self.delta=0    # reset delta
        self.pos=0	# reset absolute total position
        self.prev_pos=self.value    # store value in prev_pos again, if this is reset as well the read calculations can result in negative delta values
        


if __name__ == "__main__":  # test code contained below
    encoder1= EncoderReader('PC6','PC7',8)  # set up the encoder
    # motor1 = MotorDriver ('PA10', 'PB4', 'PB5', 3)    # can use the previous MotorDriver to run the motor for testing
    # motor1.set_duty_cycle(50) 

    # test code for the encoder
    while True:
        try:
            for i in range(100):
                encoder1.read() # read the encoder every 0.1 seconds
                print(encoder1.pos) # print the position
                time.sleep(.1)  # sleep
            encoder1.zero() # after 100 iterations test if the zero function works correctly
        except KeyboardInterrupt:   # exit if there is a Keyboard Interrupt
            break