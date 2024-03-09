import micropython
import time
import pyb

#servo controller


#intialize pin
servo_pin=pyb.Pin(pyb.Pin.board.PA1, pyb.Pin.OUT_PP, value=0)   #initialize the pin as output pin
t = pyb.Timer(2, freq=1000)                   #start the timer 

tch1 = t.channel(2,pyb.Timer.PWM, pin=servo_pin)  #start PWM on the timer channel for the pin

t.prescaler(80) #set prescaler to get clock frquency to 1 Mhz, not 80
#t.period(19999) # sets the period to be 1 ms, so pulse width can be set in units of ms

# while True :  
#     try:
#         #set the pulse width 
#         tch1.pulse_width(1500)
#         time.sleep(3)
#         tch1.pulse_width(2400)
#         time.sleep(3)
#     except KeyboardInterrupt:
#         tch1.pulse_width(0)
#         break
    
tch1.pulse_width(1500)
time.sleep(3)
tch1.pulse_width(2400)
time.sleep(3)
tch1.pulse_width(1500)
time.sleep(3)
tch1.pulse_width(0)

