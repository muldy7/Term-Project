"""!
@file main.py
    This file contains our main code to test two different motors cooperatively.
    We used some code from "basic_tasks.py" but modified it to fit our needs.
    This is the file that is stored on the Nucleo microcontroller.

    upload working code!

@author JR Ridgely, Abe Muldrow, Lucas Rambo, Peter Tomson
@date   2021-Dec-15 JRR Created from the remains of previous example
@date February 29, 2024 
@copyright (c) 2015-2021 by JR Ridgely and released under the GNU
    Public License, Version 2. 
"""

import gc
import pyb
import cotask
import task_share
from pd_controller import PD_Controller
from servo_driver import ServoDriver
from controller import Controller
from motor_driver import MotorDriver
from encoder_reader import EncoderReader
import utime
from mlx_cam import MLX_Cam
from machine import Pin, I2C


def task1_fun(shares):
    """!
    Task that controls the motion of the first motor.
    @param shares A list holding the share and queue used by this task
    """
    # Get references to the share and queue which have been passed to this task
    i_flg, cam_set = shares    # set up the local variables for the shares
    
    #setting up the FSM
    s0_init=0
    s1_spin=1
    s2_control=2
    s3_shoot=3
    s4_return=4
    s5_stop=5

    state=0

    while True:
        print(state)
        # STATE 0: INIT
        if state==s0_init:
            #instantiate the hardware 
            motor1=MotorDriver('PC1','PA0','PA1',5)

            encoder1=EncoderReader('PC6','PC7',8)   

            controller1=PD_Controller(6560,1,0.015,encoder1)   # set up initial control parameters for the 180 degree rotate

            servo1=ServoDriver('PA5',2,1)
            
            # interstate variables
            # count = 0   # for counting in the control loop

            time_start = utime.time()
            
            i_flg.put(1)    # set to one if we 
            cam_set.put(0)

            # set the next state
            state=s1_spin
            yield
        
        # STATE 1: SPIN 180 DEGREES
        elif state == s1_spin:
            # loop to control the motor to spin 180 degrees
            for i in range(150):    
                controller1.output_fun.read()    # run the controller
                meas_output=controller1.output_fun.pos   # set the measured output
                controller1.run(-1*meas_output)    # run the controller with the new measured output
                PWM=controller1.PWM # set a new PWM from the conroller run function
                motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                    
                # print out for testing
                print(i,PWM,controller1.err)
                yield
                # exit the loop once the error is small enough
                if controller1.err <= 10 and controller1.err >= -10:
                        
                    motor1.set_duty_cycle(0)
                    #utime.sleep_ms(10)	# wait 0.10 ms before moving to next state
                    controller1.reset_loop()	# reset the controller
                    controller1.output_fun.zero()	# zero the encoder
                    controller1.set_setpoint(0) # reset set point
                    i_flg.put(0)    # tell the camera we are ready for an image
                    
                    # move to the next state
                    state=s2_control	
                    #utime.sleep(3)
                    break
                elif i == 149:  # if error never gets small enough leave the loop
                    motor1.set_duty_cycle(0)
                    #utime.sleep_ms(10)	# wait 0.10 ms before moving to next state
                    controller1.reset_loop()	# reset the controller
                    controller1.set_setpoint(0)
                    controller1.output_fun.zero()	# zero the encoder
                    i_flg.put(0)    # tell the camera we are ready for an image

                    # move to the next state
                    state=s2_control	
                    #utime.sleep(3)
                    break
                else:
                    yield   # yield if still looping
            
            yield


        # STATE 2: CONTROL THE MOTOR FOR 6 SECONDS
        elif state==s2_control:

            # change gain values for the controller for small movements
            controller1.set_Kp(1.15)	
            controller1.set_Kd(.01575)

            # test prints
            print('image flag: ' + str(img_flg))
            print('setpoint: ' + str(controller1.setpoint))

            # if image in the loop isnt working we can try this
            # this works just like how it works in the other task so kinda removes point of using a scheduler
            # could be tested to get timing right
           # img_flg = i_flg.get()
           # while img_flg == 0:
           #     img_flg = i_flg.get()
           #     camera_error = cam_set.get()
           #     controller1.set_setpoint(camera_error)
           #     controller1.output_fun.zero()
           #     controller1.reset_loop()   # need to test if this is necessary or not
           #     print('waiting for image in the control loop')
           #     
           #     yield 
            
            # the control loop for the camera controller, just need to make sure it can get back here while the camera is working
            for i in range(150):
                # before controller the motor, test if there is a new image
                # delete this and try above if it doesn't work
                img_flg = i_flg.get()   # get the current image_flag value
                if img_flg == 1:
                    camera_error = cam_set.get()
                    controller1.set_setpoint(camera_error)
                    controller1.output_fun.zero()
                    controller1.reset_loop()   # need to test if this is necessary or not
                    print('changing the setpoint!')
                    print(camera_error)
                    print(encoder1.pos)
                    print(controller1.setpoint) 

                    # can try zeroing the flags here for a new image  
                    # if its working while it waits till error is small
                    #i_flg.put(0)    # tell the camera we are ready for an image
                    #img_flg = 0

                # the control loop
                controller1.output_fun.read()    # run the controller
                meas_output=controller1.output_fun.pos   # set the measured output
                controller1.run(-1*meas_output)    # run the controller with the new measured output
                PWM=controller1.PWM # set a new PWM from the conroller run function
                motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                    
                # print out for testing
                print(i,PWM,controller1.err)

                # calculate the elapsed time, want to exit the loop after 5 seconds
                curr_time = utime.time()
                elap_time = curr_time - time_start

                # exit the loop once the error is small enough
                if (controller1.err <= 10 and controller1.err >= -10) or i == 149:
                        
                    motor1.set_duty_cycle(0)
                    #utime.sleep_ms(10)	# wait 0.10 ms before moving to next state
                    controller1.reset_loop()	# reset the controller
                    controller1.output_fun.zero()	# zero the encoder
                    controller1.set_setpoint(0) # reset set point

                    # tell the camera we are ready for an image
                    i_flg.put(0)       # reset the flags
                    img_flg = 0        

                    # only exit once the error is small enough even if its been longer than 5 seconds
                    if elap_time >= 6:
                        print('shooting!')
                        
                        state=s3_shoot
                    #utime.sleep(3)
                    break
            yield


        # STATE 3: Shoot the Target       
        elif state==s3_shoot:
            
            # pull the trigger
            print('pulling the trigger!')
            servo1.set_pos(160)	# move the servo into position
            utime.sleep(1)	# wait
            servo1.set_pos(195)	# pull the trigger
            utime.sleep(1)	# wait
            servo1.set_pos(160)	# pull it back

            # set the state to the return to zero_state
            state=s4_return
            yield

        
        #STATE 4: Return to Zero
        elif state==s4_return:
            motor1.set_duty_cycle(0)    # stop the robot from moving

            # set the setpoint to negative 180 degrees
            neg_setpoint = 6560
            controller1.set_setpoint(neg_setpoint)

            # set the new gains
            controller1.set_Kp(0.1)
            controller1.set_Kd(0)

            # same loop as the 180 spin as seen before
            for i in range(150):    
                controller1.output_fun.read()    # run the controller
                meas_output=controller1.output_fun.pos   # set the measured output
                controller1.run(-1*meas_output)    # run the controller with the new measured output
                PWM=controller1.PWM # set a new PWM from the conroller run function
                motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                    
                # print out for testing
                print(i,PWM,controller1.err)
                    
                # exit the loop once the error is small enough
                if (controller1.err <= 10 and controller1.err >= -10) or i == 149:  # or once theres been a full rotation in case of steady state error
                    
                    motor1.set_duty_cycle(0)    # stop the motor
    
                    # move to the next state
                    state=s5_stop
                    break
                    #yield
            yield

            
        # STATE 5: Stop the Robot
        else:
            motor1.set_duty_cycle(0)
            
            # print final values for verification
            # print final camera_error
            print(camera_error)

            print('finished with destorying my enemies!')
            break
        
            
# TASK 2: CAMERA TASK
def task2_fun(shares):
    """!
    Task for testing the use of tasks during set up of our code
    @param shares A tuple of a share and queue from which this task gets data
    """
    # Get references to the share and queue which have been passed to this task
    i_flg_cam, cam_set_cam = shares

    state = 0

    while True:
        # STATE 0: INIT
        if state == 0:
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
            camera._camera.refresh_rate = 10.0  # can try faster refresh rates
            print(f"Refresh rate is now:  {camera._camera.refresh_rate}")
            
            # set the next state
            state=1
            yield
        
        # STATE 1: Take a picture
        else:
            img_flg = i_flg_cam.get()
            
            if img_flg == 0:   # wait for image flag to be true
            
                image = None # reset image
                while not image:
                    image = camera.get_image_nonblocking()
                    print('waiting for image')
                    #utime.sleep_ms(5) # maybe not in scheduler?
                    yield # can add state to yield? yielding doesn't break! I see 
                
                
                # test prints
                print('took a picture!')
                #utime.sleep(5)
                    
                # this code may have to go in another state we will see if it scips the yield once the image is created?
                #adjust = -15 # how much adjustment to the setpoint, could be done after get_hotspot to use the max value
                centroid = True # do the centroid calculation for our camera to find i_bar

                # find the hotspot and put it in the share 
                camera.get_hotspot(image, centroid, limits=(0, 1000))
                cam_set_cam.put(camera.camera_error)

                # lower the flag
                i_flg_cam.put(1)	# reset the shared flag variable
                img_flg = 1		# reset the local flag variable
                
                #image = None

                
                yield

            else:
                # yield and wait for the motor task
                yield
       
        

        
# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":

    # Create intertask variables for the loops
    camera_setpoint = task_share.Share('f', thread_protect=False, name="Camera Setpoint Share")    # share for the camera_error variable, the setpoint from the hotspot, f for float
    image_flag = task_share.Share('h', thread_protect=False, name="Image Flag Share")      # flag for communicating if an image is required

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for alw
    # debugging and set trace to False when it's not needed

    # task1 is the motor and servo controlling task
    task1 = cotask.Task(task1_fun, name="Motor and Servo Control", priority=1, period=5,    # could be much lower for our loop depending on precision
                        profile=True, trace=False, shares=(image_flag, camera_setpoint))
    
    # task2 is the imager for the camera
    task2 = cotask.Task(task2_fun, name="Thermal Camera Imager", priority=2, period=100, # should experiment with the period
                        profile=True, trace=False, shares=(image_flag, camera_setpoint))
    
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)
    

    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

    # Print a table of task data and a table of shared information data
    print('\n' + str (cotask.task_list))
    print(task_share.show_all())
    print(task1.get_trace())
    print('')

