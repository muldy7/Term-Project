'''!
@file pd_controller.py

This file attempts to implement pid control of our device with some small changes to the previous controller class

I guess we could put all three controllers in ONE file that holds the three different encoder classes?

This file contains code to create a controller class that allows us to create a control loop to run our motor.
This controller class when used in conjuction with our motor_driver and encoder_reader classes allow use to implement 
closed loop porportional control of our motor. The class below includes function to init and run out controller, 
along with setting the setpoint and Kp value of the control loop. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date 3/8/2024  
'''
# changing the date would allow us to do a good job taking note of changes

import time

class PID_Controller:
    
    def __init__ (self, setpoint, Kp, Kd, Ki, read_fun):
        """! 
        Creates a Controller Object by initializing a setpoint (a desired location for the motor)
        a Kp value given by a user, and a encoder read function from an Encoder Object.

        @param setpoint This is a location for the motor in units of encoder counts.
        @param Kp A control gain set by the user. This is used to produce the actuation signal.
        @param read_fun This is a read function from an Encoder Object.

        @param Kd this is the derivative gain for our controller
        @param Ki
        """
        self.Kp=Kp
        self.Kd = Kd
        self.Ki = Ki    # new can give a value of zero for testing
        self.setpoint=setpoint
        self.output_fun=read_fun
        self.pos_output=[]
        self.T = 0.01   # this is the trigger rate for the controller, going to assume like 1 ms but we can test
        self.delta_err = 0 # this store the change in error for derivative control, the first value should be error
        self.err = 0
        self.prev_err = 0 # previous error value for delta_err calculation
        self.tot_err = 0
        
    def run(self,meas_output):
        """!
        This function runs the controller and changes the PWM for the next cycle.
    
        @param meas_output This value is the previous measured output.
        """
        # create the equation that runs the control loop
        self.err = self.setpoint - meas_output
        self.delta_err = self.err - self.prev_err
        self.PWM=self.Kp*(self.err) + self.Kd*(self.delta_err/self.T) + self.Ki*(self.tot_err*self.T)
        self.pos_output.append(meas_output) # store value for printing
        self.prev_err = self.err    # store this error as previous error
        self.tot_err += self.err    # add the last error to total error
    
    def set_setpoint(self,setpoint):
        """!
        This function sets the setpoint for the controller object.
    
        @param setpoint This is a location for the motor in units of encoder counts.
        """
        self.setpoint=setpoint  # store in the class object
        
    def set_Kp(self,Kp):
        """!
        This function sets the Kp value for the controller object.
    
        @param Kp A control gain set by the user. This is used to produce the actuation signal.
        """
        self.Kp=Kp

    def set_Kp(self,Kd):
        """!
        This function sets the Kd value for the controller object.
    
        @param Kd A control gain set by the user. This is used to produce the actuation signal.
        """
        self.Kd=Kd

    def set_Ki(self,Ki):
        """!
        This function sets the Ki value for the controller object.
    
        @param Ki A control gain set by the user. This is used to produce the actuation signal.
        """
        self.Ki=Ki
        
    def step_response(self):
        print('start')
        for i in range(len(self.pos_output)):
            print(i*10,self.pos_output[i])
        print('end')
        
    def reset_loop(self):
        """!
        This function resets the calculations for a new control loop. 
        This function can be used in conjuction with encoderX.read() & set_setpoint() for a new control loop of a different angle

        """
        self.delta_err = 0 # this store the change in error for derivative control, the first value should be error
        self.prev_err = 0 # previous error value for delta_err calculation
        self.tot_err = 0 # reset tot_err

        # could also have it call encoderX.zero() so it's done in one function?
        self.output_fun.zero()  # we can try and see if works

        # could add setpoint here as well?


if __name__ == "__main__":  # test code contained below
    encoder1 = []
    controller1 = PID_Controller(100,.1,.1,0.0001,encoder1) # i guess we wont want PID becuase I will never be zero
    total_pwm = 0
    run = 1 # only want to print set_time once

    for i in range(150):
            time.sleep(0.01)
            #utime.sleep_ms(10)  # sleep for 10 ms
            #controller1.output_fun.read()    # run the controller
              # set the measured output
            if i >= 100:
                controller1.run(100)    # run the controller with the new measured output
            else:
                controller1.run(i)

            print(i, controller1.PWM)
            total_pwm = total_pwm + controller1.PWM # calc the total PWM

            if controller1.PWM == 0 and run == 1:
                set_time = i    # denote what the settling time is when pwm equals 0
                print(set_time)
                run = 0

    avg_PWM = total_pwm/set_time
    print('AVG PWM Value: ' + str(avg_PWM))
    print('Settling Time: ' + str(set_time))
   # controller1.reset_loop()