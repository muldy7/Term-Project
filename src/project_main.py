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
s0_init=0
s1_rotate=1
s2_control=2
s3_shoot=3
s4_stop=4

state=0

while True:
        print('state: ' + str(state))
      
                # STATE 0: INIT
        if state==s0_init:
            #instantiate the hardware 
            motor1=MotorDriver('PC1','PA0','PA1',5)

            encoder1=EncoderReader('PC6','PC7',8)   

            controller1=PD_Controller(6560,.4,0.0185,encoder1)

            servo1=ServoDriver('PA5',2,1)
        
            # camera set-up code
            try:
                # The following import is only used to check if we have an STM32 board such
                # as a Pyboard or Nucleo; if not, use a different library
                from pyb import info    

                # Oops, it's not an STM32; assume generic machine.I2C for ESP32 and others
            except ImportError:
                    # For ESP32 38-pin cheapo board from NodeMCU, KeeYees, etc.
                i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))

                # OK, we do have an STM32, so just use the default pin assignments for I2C1
            else:
                i2c_bus = I2C(1)

            print("MXL90640 Easy(ish) Driver Test")

            # Select MLX90640 camera I2C address, normally 0x33, and check the bus
            i2c_address = 0x33
            scanhex = [f"0x{addr:X}" for addr in i2c_bus.scan()]
            print(f"I2C Scan: {scanhex}")

                # Create the camera object and set it up in default mode
            camera = MLX_Cam(i2c_bus)
            print(f"Current refresh rate: {camera._camera.refresh_rate}")
            camera._camera.refresh_rate = 10.0
            print(f"Refresh rate is now:  {camera._camera.refresh_rate}")
            
            
            state=s1_rotate
        
        # STATE 1: Rotate 180 Degrees
        elif state==s1_rotate:

    
                for i in range(150):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
                    utime.sleep_ms(10)
                    controller1.output_fun.read()    # run the controller
                    meas_output=controller1.output_fun.pos   # set the measured output
                    controller1.run(-1*meas_output)    # run the controller with the new measured output
                    PWM=controller1.PWM # set a new PWM from the conroller run function
                    motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                    
                    # print out for testing
                    print(i,PWM,controller1.err)
                    
                    # exit the loop once the error is small enough
                    if controller1.err <= 10 and controller1.err >= -10:
                        
                        motor1.set_duty_cycle(0)
                        utime.sleep_ms(10)	# wait 0.10 ms before moving to next state
                        controller1.reset_loop()	# reset the controller
                        controller1.output_fun.zero()	# zero the encoder
                        
                        state=s2_control	# move to the next state
                        utime.sleep(1)
                        break
                    if i == 149:
                        motor1.set_duty_cycle(0)
                        utime.sleep_ms(10)	# wait 0.10 ms before moving to next state
                        controller1.reset_loop()	# reset the controller
                        controller1.output_fun.zero()	# zero the encoder
                        
                        state=s2_control	# move to the next state
                        utime.sleep(1)
                        break
                    
        
        # STATE 2: Find the Target
        elif state==s2_control:
                # Keep trying to get an image
                image = None
                while not image:
                        image = camera.get_image_nonblocking()
                        utime.sleep_ms(5) #50
                    
                # calculate the centroid if centroid = True
                centroid = True # do the centroid calculation for our camera to find i_bar
                
                # calculate the hotspot from the camera image
                camera.get_hotspot(image, centroid, limits=(0, 1000))	# should we do a smaller value for calculations? is bigger or smaller better
                
                # set the controller setpoint to the calculated encoder tics
                controller1.set_setpoint(camera.camera_error)	# camera.error is the calculated tics from the get_hotspot function
                
                # set gain values, can be fiddled with
                controller1.set_Kp(1.1)	
                controller1.set_Kd(.0158)
            
                # start the control loop
            
                for i in range(150):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
                    utime.sleep_ms(1)
                    controller1.output_fun.read()    # run the controller
                    meas_output=controller1.output_fun.pos   # set the measured output
                    controller1.run(-1*meas_output)    # run the controller with the new measured output
                    PWM=controller1.PWM # set a new PWM from the conroller run function
                    motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                    print('targeting:' + str(i) + ',' + str(controller1.err) + ',' + str(PWM))
                    
                    # exit once the error is small enough, this could be refined as well
                    print(controller1.err)
                    if controller1.err <= 10 and controller1.err >= -10:
                        motor1.set_duty_cycle(0)	# stop the motor
                        utime.sleep_ms(20)
                        
                        # branch to state_3
                        state=s3_shoot
                        break
            
                
        # STATE 3: Shoot the Target       
        elif state==s3_shoot:   
                # pull the trigger 
                servo1.set_pos(160)	# move the servo into position
                utime.sleep(1)	# wait
                servo1.set_pos(195)	# pull the trigger
                utime.sleep(1)	# wait
                servo1.set_pos(160)	# pull it back
                state = s4_stop
        
        # STATE 4: Stop the Robot
        elif state==s4_stop:
                motor1.set_duty_cycle(0)	# stop the motor
                
                controller1.set_Kp(.1)
                controller1.set_Kd(0)
                controller1.set_setpoint(-1*(6560+(controller1.setpoint)))	# set the setpoint to the total encoder tics moved
                
                #return the device to zero
                for i in range(300):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
#                         #print(i)
#                         #utime.sleep_ms(1)
#                         utime.sleep_ms(10)
#                         controller1.output_fun.read()    # run the controller
#                         meas_output=controller1.output_fun.pos   # set the measured output
#                         controller1.run(-1*meas_output)    # run the controller with the new measured output
#                         PWM=controller1.PWM # set a new PWM from the conroller run function
#                         motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                        
                        # exit once error is small enough
                        if controller1.err <= 10 and controller1.err >= -10:
                            motor1.set_duty_cycle(0)
                            break
                
                # print values for verification and testing
                #print(camera.avg)	# print the list of avg_sums
                print('i_bar: ' + str(camera.i_bar))	# print i_bar
                print('i_max: ' + str(camera.max_index))
                print('camera_error (tics): ' + str(camera.camera_error)) # this is an encoder tic value
                print('setpoint (should be equal to cam_err): ' + str(controller1.setpoint))
                print('final enc pos: ' + str(controller1.output_fun.pos))
                
                
                # calculate values
                print(camera.sums)
                theta_act = controller1.output_fun.pos*(180/6600) # calculate the actual degrees turned
                theta_exp = camera.i_bar*(55/32) # calculate the expected degrees to turn from the temp calc
                ss_error = (camera.camera_error + controller1.output_fun.pos)
                
                print('theta actual: ' + str(theta_act))
                print('theta expected: ' + str(theta_exp))
                print('steady state error: ' + str(ss_error))
                
                break


        