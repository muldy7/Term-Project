"""!
@file nucleo_main.py

This has mostly been converted to a test code we should probably grab a main.py from lab3 or lab4 for testing 

This file contains code that was stored on our group's Nucleo board to generate the motor step-response. 
This file was called by the code on our computer and read through the serial port to generate graphs of our step-response.
The file below imports the three different classes we had previoulsy set up to run the motor. 
The code below sets ups the motor, encoder, and controller classes necassary with their various pins. 
After initilization, the code then starts a loop where it asks for a Kp value and then runs the given Kp value in a motor step-response. 
Our motor_driver and encoder_reader classes were importanted to the board as well. Copies of these files can be found in the source code. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
"""

from pd_controller import PD_Controller
from controller import Controller
from motor_driver import MotorDriver
from encoder_reader import EncoderReader

import utime


motor1=MotorDriver('PC1','PA0','PA1',5)

encoder1=EncoderReader('PC6','PC7',8)

controller1=PD_Controller(733,.1,0.001,encoder1)  # one rotation of the device is 4 rotations of the motor, 
                                        # so 180 degrees is 2X whatever how many encoder tics two rotations are
controller2=Controller(6600,1,encoder1)

while True:
    print('awaiting input') # send to computer to alert the board is ready for a Kp value
    Kp=float(input('Input a Value for Kp: '))   # await a Kp value from the computer
    controller1.set_Kp(Kp)  # set the controller Kp value
    Kd=float(input('Input a Value for Kd: '))   # await a Kd value from the computer
    controller1.set_Kd(Kd)
    set_point=float(input('Input a Setpoint: '))   # await a Kp value from the computer
    controller1.set_setpoint(set_point)  # set the controller Kp value
    
    #motor1.set_duty_cycle(-50)
    try:
        start_time = utime.time_ns()
        for i in range(300):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
            #utime.sleep_us(1)  # sleep for 10 ms
            controller1.output_fun.read()    # run the controller
            meas_output=controller1.output_fun.pos   # set the measured output
            controller1.run(-1*meas_output)    # run the controller with the new measured output
            PWM=controller1.PWM # set a new PWM from the conroller run function
           
            #print(utime.ticks_us())
            # values for testing
            #print(meas_output)
            #print('Encoder Value: ' + str(meas_output)) # these prints will mess up the GUI just for testing
            #print('PWM: ' + str(controller1.PWM))

            motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
        end_time = utime.time_ns()
        total_time = end_time - start_time
        
        print(total_time)
        controller1.output_fun.read()	# read the final value of the encoder
        print(controller1.setpoint)
        print(controller1.output_fun.pos)
        ss_error = float(controller1.setpoint) + float(controller1.output_fun.pos) # this should give us the steady state error (plus becuase encoders are swapped?)

        print('Steady State Error: ' + str(ss_error))    # this value will be in encoder tics
        print('yay!')
        controller1.output_fun.zero()    # at the end of the loop disable the motor, output_fun is encoder1
        motor1.set_duty_cycle(0)
        controller1.reset_loop()
        
            # controller1.step_response() # run out function that prints the values of the step response to the computer
    except KeyboardInterrupt:
         motor1.set_duty_cycle(0)
         break
    except TypeError:
         print('TypeError')
         motor1.set_duty_cycle(0)
         break
        
    
    
        
    

            
        