"""!
@file nucleo_main.py

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
from servo_driver import ServoDriver
from controller import Controller
from motor_driver import MotorDriver
from encoder_reader import EncoderReader
import utime
from mlx_cam import MLX_Cam
import gc
from machine import Pin, I2C


#setting up the FSM
while True:
        s0_init=0
        s1_rotate=1
        s2_control=2
        s3_shoot=3
        s4_stop=4
        
        state=0
        try:
                if state==s0_init:
                        #instantiate the hardware 
                        motor1=MotorDriver('PC1','PA0','PA1',5)

                        encoder1=EncoderReader('PC6','PC7',8)   

                        controller1=PD_Controller(0,1,0.015,encoder1)

                        servo1=ServoDriver('PA1',2) 

                        state=s1_rotate
                
                elif state==s1_rotate:

                        try:

                                for i in range(150):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
                                        controller1.output_fun.read()    # run the controller
                                        meas_output=controller1.output_fun.pos   # set the measured output
                                        controller1.run(-1*meas_output)    # run the controller with the new measured output
                                        PWM=controller1.PWM # set a new PWM from the conroller run function
                                        motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                                controller1.reset_loop()
                                state=s2_control
                                # controller1.step_response() # run out function that prints the values of the step response to the computer
                        except KeyboardInterrupt:
                                motor1.set_duty_cycle(0)
                                break
                        except TypeError:
                                print('TypeError')
                                motor1.set_duty_cycle(0)
                                break

                elif state==s2_control:
                        # Keep trying to get an image; this could be done in a task, with
                        # the task yielding repeatedly until an image is available
                        utime.sleep(2)
                        image = None
                        while not image:
                                image = camera.get_image_nonblocking()
                                utime.sleep_ms(5) #50
                                camera.get_hotspot(image, limits=(0, 1000))
                                controller1.set_setpoint(camera.camera_error)
                                break
                        state=s3_shoot        
                elif state==s3_shoot:   
                        #pull the trigger 
                        ServoDriver.set_pos(122) 
                        state=s4_stop
                elif state==s4_stop:
                        motor1.set_duty_cycle(0)

        except KeyboardInterrupt:
                motor1.set_duty_cycle(0)
                break
        except TypeError:
                print('TypeError')
                motor1.set_duty_cycle(0)
                break
