"""!
@file main.py

THIS IS THE FILE ON THE NUCLEO BOARD

This file contains code that was stored on our group's Nucleo board to generate the motor step-response. 
This file was called by the code on our computer and read through the serial port to generate graphs of our step-response.
The file below imports the three different classes we had previoulsy set up to run the motor. 
The code below sets ups the motor, encoder, and controller classes necassary with their various pins. 
After initilization, the code then starts a loop where it asks for a Kp, Kd, and setpoint value and then 
runs the controller with the given values in a motor step-response. 

Our motor_driver, pd_controller, and encoder_reader classes were importanted to the board as well. Copies of these files can be found in the source code. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson

@date March 8th, 2024
"""

from pd_controller import PD_Controller
from motor_driver import MotorDriver
from encoder_reader import EncoderReader
import utime

# set up the classes
motor1=MotorDriver('PC1','PA0','PA1',5)

encoder1=EncoderReader('PC6','PC7',8)

controller1=PD_Controller(6600,1,.1,encoder1)

# run the step response
while True:
    print('awaiting input') # send to computer to alert the board is ready for a gain values
    Kp=float(input('Input a Value for Kp: '))   # await a Kp value from the computer
    controller1.set_Kp(Kp)  # set the controller Kp value
    Kd=float(input('Input a Value for Kd: '))   # await a Kd value from the computer
    controller1.set_Kd(Kd)  # set the controller Kd value
    set_point=float(input('Input a Setpoint: '))   # ask for a setpoint from the computer
    controller1.set_setpoint(set_point)  # set the controller setpoint
    
    try:
        for i in range(300):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
            #utime.sleep_ms(1)  # sleep for 1 ms
            controller1.output_fun.read()    # run the controller
            meas_output=controller1.output_fun.pos   # set the measured output 
            controller1.run(-1*meas_output)    # run the controller with the new measured output, have to convert the signs because encoder wires are switch
            PWM=controller1.PWM # set a new PWM from the conroller run function
            motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
            
        controller1.output_fun.zero()    # at the end of the loop disable the motor, output_fun is encoder1
        motor1.set_duty_cycle(0)
        controller1.step_response() # run out function that prints the values of the step response to the computer
    except KeyboardInterrupt:
         motor1.set_duty_cycle(0)
         break

            
        
