import micropython
import time
import pyb


class Servo_Driver:
    def __init__ (self, servo_pin, timer_channel):

        servo_pin = getattr(pyb.Pin.board, servo_pin) # get the pin value for the pin store in en_pin
        timer_channel = getattr(pyb.Pin.board, timer_channel)
        
    
        self.servo_pin=pyb.Pin(servo_pin, pyb.Pin.OUT_PP, value=0)   #initialize the pin as output pin
        t = pyb.Timer(timer_channel, freq=1000) #start the timer 
        t.prescaler(80) #set prescaler to get clock frquency to 1 Mhz, not 80
        self.timer_channel=t.channel(timer_channel,pyb.Timer.PWM, pin=self.servo_pin)  #start PWM on the timer channel for the pin


                 
    def set_pos(self,angle):
        PWM_angle=(angle/180)*2000+500     #converts an angle in degrees to the pwm needed for the servo controller. 500 microseconds 
                                            # is 0 degrees, 2500 is 180 degrees
        self.timer_channel.pulse_width(PWM_angle)


