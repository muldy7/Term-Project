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

from controller import Controller
from motor_driver import MotorDriver
from encoder_reader import EncoderReader
import utime


motor1=MotorDriver('PC1','PA0','PA1',5)

encoder1=EncoderReader('PC6','PC7',8)

controller1=Controller(733,1,encoder1)  # one rotation of the device is 4 rotations of the motor, 
                                        # so 180 degrees is 2X whatever how many encoder tics two rotations are


while True:
    print('awaiting input') # send to computer to alert the board is ready for a Kp value
    Kp=float(input('Input a Value for Kp: '))   # await a Kp value from the computer
    controller1.set_Kp(Kp)  # set the controller Kp value
    set_point=float(input('Input a Setpoint: '))   # await a Kp value from the computer
    controller1.set_setpoint(set_point)  # set the controller Kp value
    
    #motor1.set_duty_cycle(-50)
    try:
#         for i in range(100):
#             encoder1.read() # read the encoder every 0.1 seconds
#             #print(encoder1.pos) # print the position
#             utime.sleep(.1)  # sleep
#             controller1.output_fun.read()    # run the controller
#             meas_output=controller1.output_fun.pos
#             print('Encoder Value: ' + str(meas_output))
#             utime.sleep(1)
#             controller1.run(meas_output)
#             print('PWM: ' + str(controller1.PWM))
#             utime.sleep(1)
            #encoder1.zero()
        for i in range(300):    # each run should be 3000 miliseconds or 3 seconds, each run acts as a controller loop
            utime.sleep_ms(10)  # sleep for 10 ms
            controller1.output_fun.read()    # run the controller
            meas_output=controller1.output_fun.pos   # set the measured output
            controller1.run(-1*meas_output)    # run the controller with the new measured output
            PWM=controller1.PWM # set a new PWM from the conroller run function

            print(meas_output)
            print('Encoder Value: ' + str(meas_output)) # these prints will mess up the GUI just for testing
            print('PWM: ' + str(controller1.PWM))

            motor1.set_duty_cycle(PWM)  # set the PWM of the motor to the new PWM value

        # test print for the steady state error
        ss_error = controller1.setpoint - controller1.output_fun.read() # this should give us the steady state error

        print('Steady State Error: ' + ss_error)    # this value will be in encoder tics

        controller1.output_fun.zero()    # at the end of the loop disable the motor, output_fun is encoder1
        motor1.set_duty_cycle(0)
#         controller1.step_response() # run out function that prints the values of the step response to the computer
    except KeyboardInterrupt:
         motor1.set_duty_cycle(0)
         break

            
        
