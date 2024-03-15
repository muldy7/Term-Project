"""!
@file main.py
    This file contains our main code to test two different motors cooperatively.
    We used some code from "basic_tasks.py" but modified it to fit our needs.
    This is the file that is stored on the Nucleo microcontroller.

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
    image_flag, camera_setpoint = shares    # not sure if I did this correctly
    
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

            controller1=PD_Controller(6560,.9,.02,encoder1)   # set up initial control parameters for the 180 degree rotate

            servo1=ServoDriver('PA5',2,1)

            # start to servo to prevent from shooting
            #servo1.set_pos(60)  # may not need this?

            # interstate variables
            count = 0   # for counting in the control loop
            total_ticks = 0
            
            # set the next state
            state=s1_control
            yield
        
        elif state==s1_control:

            
            print(controller1.err)
            
            # controller calculations
            controller1.output_fun.read()    # run the controller
            meas_output=controller1.output_fun.pos   # set the measured output
            controller1.run(-1*meas_output)    # run the controller with the new measured output
            
            if controller1.err >= 5 or controller1.err <= -5: # code to exit the loop once error is small enough
                #utime.sleep_ms(1)  # can set a different amount to change the amount of control
                #utime.sleep_ms(10)
                print('controlling')
                PWM=controller1.PWM # set a new PWM from the conroller run function
                motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                #print(i,PWM)
                #print('targeting:' + str(i) + ',' + str(controller1.err) + ',' + str(PWM))
                yield
            else:
                print('done controlling')
                motor1.set_duty_cycle(0)    # stop motor
                #utime.sleep_ms(10)
                if count == 0:  # first pass of 180 degrees
                    state=s2_image
                    image_flag.put(1) # set the image flag
                    count += 1
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    controller1.reset_loop()    # reset the loop and encoder
                    yield   # this should be fin as a yield
                elif count == 1:    # second control pass for finding target (could technically change ifs)
                    count += 1
                    state=s2_image
                    image_flag.put(1)   # set the image flag
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    controller1.reset_loop()   # reset the loop and encoder
                    yield
                elif count==2:       # after third control loop for finding target
                    count += 1
                    total_ticks += controller1.output_fun.pos   # add encoder ticks of first rotation
                    state=s3_shoot  
                    controller1.reset_loop()
                    yield
                else:
                    state=s5_stop   # final loop of returning to zero 
                    yield
            
        # STATE 2: Find the Target
        elif state==s2_image:
                
                camera_error = camera_setpoint.get()
                #print(camera_error)
                if camera_error <= 0 or camera_error >= 0:    # wait for a camera error value
                    print('waiting for setpoint')
                    #camera_error = camera_setpoint.get()
                    print(camera_error)
                    yield
                else:
                    # values for testing
                    adjust = -15 # how much adjustment to the setpoint, could be done after get_hotspot to use the max value
        
                    controller1.set_setpoint(camera_error+adjust+controller1.setpoint)	# add camera error to previous setpoint
                    
                    # set gain values, can be fiddled with
                    #controller1.set_Kp(1)	
                    #controller1.set_Kd(.002)

                    state = s1_control
                    camera_error.put(0) # reset camera_error    # not sure if I do this here?
                    camera_error = 0	# reset local variable
                    yield

        # STATE 3: Shoot the Target       
        elif state==s3_shoot:   
            # pull the trigger 
            servo1.set_pos(160)
            time.sleep(1)
            servo1.set_pos(195)
            time.sleep(1)
            servo1.set_pos(160)

            state=s4_return
            yield
        
        #STATE 4: Return to Zero
        elif state==s4_return:
            controller1.set_setpoint(-1*controller1.setpoint) # set the setpoint to the opposite of total ticks
            controller1.set_Kd(0)
            controller1.set_Kp(.01)    # set to low Kp so it returns slowly
            state=s1_control
            yield

        # STATE 5: Stop the Robot
        else:
            motor1.set_duty_cycle(0)
            #controller1.set_setpoint((6600+camera.camera_error)*-1)
            
            # print values for verification
            #print(camera.avg)	# print the list of avg_sums
            #print('i_bar: ' + str(camera.i_bar))	# print i_bar
            #print('i_max: ' + str(camera.max_index))
            #print('camera_error (tics): ' + str(camera.camera_error)) # this is an encoder tic value
            #print('setpoint (should be equal to cam_err): ' + str(controller1.setpoint))
            #print('final enc pos: ' + str(controller1.output_fun.pos))
            
            
            # calculate values
            theta_act = controller1.output_fun.pos*(6600/180) # calculate the actual degrees turned
            #theta_exp = i_bar*(55/32) # calculate the expected degrees to turn from the temp calc
            #ss_error = (camera.camera_error - controller1.output_fun.pos)
            
            print('theta actual: ' + str(theta_act))
            #print('theta expected: ' + str(theta_exp))
            #print('steady state error: ' + str(ss_error))
            
            print('finished with destorying my enemies!')
            break
        
            

def task2_fun(shares):
    """!
    Task for testing the use of tasks during set up of our code
    @param shares A tuple of a share and queue from which this task gets data
    """
    # Get references to the share and queue which have been passed to this task
    image_flag, camera_setpoint = shares

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
            camera._camera.refresh_rate = 10.0
            print(f"Refresh rate is now:  {camera._camera.refresh_rate}")
            
            # set the next state
            state=1
            yield
        
        # STATE 1: Take a picture
        else:
            image_flg = image_flag.get()
            
            if image_flg == 1:   # wait for image flag to be true
                image = None # reset image
                while not image:
                    image = camera.get_image_nonblocking()
                    #utime.sleep_ms(5) # maybe not in scheduler?
                    yield # can add state to yield? yielding doesn't break! I see 
                
                    
                    
                # this code may have to go in another state we will see if it scips the yield once the image is created?
                #adjust = -15 # how much adjustment to the setpoint, could be done after get_hotspot to use the max value
                centroid = True # do the centroid calculation for our camera to find i_bar

                # find the hotspot and put it in the share 
                camera.get_hotspot(image, centroid, limits=(0, 1000))
                camera_setpoint.put(camera.camera_error)

                # lower the flag
                image_flag.put(0)	# reset the shared flag variable
                image_flg = 0		# reset the local flag variable

                
                yield

            else:
                # yield and wait for the motor task
                yield
        yield
        

        
# This code creates a share, a queue, and two tasks, then starts the tasks. The
# tasks run until somebody presses ENTER, at which time the scheduler stops and
# printouts show diagnostic information about the tasks, share, and queue.
if __name__ == "__main__":

    # Create intertask variables for the loops
    camera_setpoint = task_share.Share('f',0)    # share for the camera_error variable, the setpoint from the hotspot, f for float
    image_flag = task_share.Share('h',0)      # flag for communicating if an image is required

    # Create the tasks. If trace is enabled for any task, memory will be
    # allocated for state transition tracing, and the application will run out
    # of memory after a while and quit. Therefore, use tracing only for 
    # debugging and set trace to False when it's not needed
    task1 = cotask.Task(task1_fun, name="Motor and Servo Control", priority=1, period=10,
                        profile=True, trace=False, shares=(image_flag, camera_setpoint))
    task2 = cotask.Task(task2_fun, name="Thermal Camera Imager", priority=2, period=10,
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

