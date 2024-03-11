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
s1_control=1
s2_image=2
s3_shoot=3
s4_return=4
s5_stop=5

state=0

while True:
      
    # STATE 0: INIT
    if state==s0_init:
        #instantiate the hardware 
        motor1=MotorDriver('PC1','PA0','PA1',5)

        encoder1=EncoderReader('PC6','PC7',8)   

        controller1=PD_Controller(6600,.1,0,encoder1)   # set up initial control parameters for the 180 degree rotate

        servo1=ServoDriver('PA5',2,1)

        # start to servo to prevent from shooting
        servo1.set_pos(60)

        # interstate variables
        count = 0   # for counting in the control loop
        total_ticks = 0

    
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
        
        # set the next state
        state=s1_control
    
    elif state==s1_control:
        for i in range(150):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
            #utime.sleep_ms(1)  # can set a different amount to change the amount of control
            #utime.sleep_ms(10)
            controller1.output_fun.read()    # run the controller
            meas_output=controller1.output_fun.pos   # set the measured output
            controller1.run(-1*meas_output)    # run the controller with the new measured output
            PWM=controller1.PWM # set a new PWM from the conroller run function
            motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
            #print(i,PWM)
            #print('targeting:' + str(i) + ',' + str(controller1.err) + ',' + str(PWM))
                    
            # code to exit the loop once error is small enough
            if controller1.err <= 2 and controller1.err >= -2:
                motor1.set_duty_cycle(0)    # stop motor
                #utime.sleep_ms(10)
                if count == 0:  # first pass of 180 degrees
                    state=s2_image
                    count += 1
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    controller1.reset_loop()    # reset the loop and encoder
                    break
                elif count == 1:    # second control pass for finding target (could technically change ifs)
                    count += 1
                    state=s2_image
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    controller1.reset_loop()   # reset the loop and encoder
                    break
                elif count==2:       # after third control loop for finding target
                    count += 1
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    state=s3_shoot  
                    controller1.reset_loop()
                    break
                else:
                    state=s5_stop   # final loop of returning to zero 
                    break
        
    # STATE 2: Find the Target
    elif state==s2_image:
            # Keep trying to get an image; this could be done in a task, with
            # the task yielding repeatedly until an image is available
            #utime.sleep(1)
            image = None
            while not image:
                    image = camera.get_image_nonblocking()
                    utime.sleep_ms(5) #50
                    
                    #break
            # values for testing
            adjust = -15 # how much adjustment to the setpoint, could be done after get_hotspot to use the max value
            centroid = True # do the centroid calculation for our camera to find i_bar
            
            camera.get_hotspot(image, centroid, limits=(0, 1000))	# should we do a smaller value for calculations? is bigger or smaller better
            controller1.set_setpoint(camera.camera_error+adjust)	# adding a little error
            
            # set gain values, can be fiddled with
            controller1.set_Kp(1)	
            controller1.set_Kd(.002)

            state = s1_control
            
    # STATE 3: Shoot the Target       
    elif state==s3_shoot:   
        # pull the trigger 
        servo1.set_pos(160)
        utime.sleep(3)
        servo1.set_pos(105)

        state=s4_return
    
    #STATE 4: Return to Zero
    elif state==s4_return:
         controller1.set_setpoint(-total_ticks) # set the setpoint to the opposite of total ticks
         controller1.set_Kd(0)
         controller1.set_Kp(.01)    # set to low Kp so it returns slowly

         state=s1_control

    # STATE 5: Stop the Robot
    else:
        motor1.set_duty_cycle(0)
        #controller1.set_setpoint((6600+camera.camera_error)*-1)
        
        # print values for verification
        print(camera.avg)	# print the list of avg_sums
        print('i_bar: ' + str(camera.i_bar))	# print i_bar
        print('i_max: ' + str(camera.max_index))
        print('camera_error (tics): ' + str(camera.camera_error)) # this is an encoder tic value
        print('setpoint (should be equal to cam_err): ' + str(controller1.setpoint))
        print('final enc pos: ' + str(controller1.output_fun.pos))
        
        
        # calculate values
        theta_act = controller1.output_fun.pos*(6600/180) # calculate the actual degrees turned
        theta_exp = i_bar*(55/32) # calculate the expected degrees to turn from the temp calc
        ss_error = (camera.camera_error - controller1.output_fun.pos)
        
        print('theta actual: ' + str(theta_act))
        print('theta expected: ' + str(theta_exp))
        print('steady state error: ' + str(ss_error))
        
        break
            



        