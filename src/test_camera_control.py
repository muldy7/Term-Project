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
from mlx_cam import MLX_Cam
import gc
from machine import Pin, I2C
#from mlx90640 import MLX90640
#from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
#from mlx90640.image import ChessPattern, InterleavedPattern


motor1=MotorDriver('PC1','PA0','PA1',5)

encoder1=EncoderReader('PC6','PC7',8)

controller1=PD_Controller(0,.1,0,encoder1)  # one rotation of the device is 4 rotations of the motor, 
                                        # so 180 degrees is 2X whatever how many encoder tics two rotations are
#controller2=Controller(6600,1,encoder1)

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

utime.sleep(3)

while True:
    try:
        # Keep trying to get an image; this could be done in a task, with
        # the task yielding repeatedly until an image is available
        utime.sleep(2)
        image = None
        while not image:
            image = camera.get_image_nonblocking()
            utime.sleep_ms(5) #50
        camera.get_hotspot(image, limits=(0, 1000))
        controller1.set_setpoint(camera.camera_error)
        
        print('taking an image')
        #print(camera.avg)
        #print(camera.max_index)
        #print(camera.max_value)
        print(camera.camera_error)
        print(camera.i_bar)
        
        #utime.sleep()
        
        #break
        
        
        
        try:
            for i in range(300):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
                utime.sleep_ms(10)
                controller1.output_fun.read()    # run the controller
                meas_output=controller1.output_fun.pos   # set the measured output
                controller1.run(-1*meas_output)    # run the controller with the new measured output
                PWM=controller1.PWM # set a new PWM from the conroller run function
                motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value
                print(controller1.err)
            
            controller1.reset_loop()
            motor1.set_duty_cycle(0)
            
            
            # controller1.step_response() # run out function that prints the values of the step response to the computer
        except KeyboardInterrupt:
             motor1.set_duty_cycle(0)
             break
    except TypeError:
         print('TypeError')
         motor1.set_duty_cycle(0)
         break
        
    
    
        
    

            
        
