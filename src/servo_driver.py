import micropython
import time
import pyb


class ServoDriver:
    def __init__ (self, servo_pin, timer, timer_channel ):

        servo_pin = getattr(pyb.Pin.board, servo_pin) # get the pin value for the pin store in en_pin
        #timer_channel = getattr(pyb.Pin.board, timer_channel)
        
    
        self.servo_pin=pyb.Pin(servo_pin, pyb.Pin.OUT_PP, value=0)   #initialize the pin as output pin
        t = pyb.Timer(timer, freq=1000) #start the timer 
        t.prescaler(80) #set prescaler to get clock frquency to 1 Mhz, not 80
        self.timer_channel=t.channel(timer_channel,pyb.Timer.PWM, pin=self.servo_pin)  #start PWM on the timer channel for the pin


                 
    def set_pos(self,angle):
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
    
    