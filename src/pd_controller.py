'''!
@file pd_controller.py

This file implements pd control of our device with some small changes to the previous controller class. 
The Kd control is implemented with an addition to the control equation, along with code to calculate the
previous and change in error for Kd implementation. 

This file contains code to create a controller class that allows us to create a control loop to run our motor.
This controller class when used in conjuction with our motor_driver and encoder_reader classes allow use to implement 
closed loop porportional and derivative control of our motor. The class below includes function to init and run our controller, 
along with setting the setpoint, Kd, and Kp value of the control loop. An additional function is included to reset the controller
values for multiple control loops during our robot's use. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date 3/8/2024  
'''

class PD_Controller:
    
    def __init__ (self, setpoint, Kp, Kd, read_fun):
        """! 
        Creates a Controller Object by initializing a setpoint (a desired location for the motor)
        a Kp value given by a user, a Kd value given by a user, and a encoder read function from an Encoder Object.

        @param setpoint This is a location for the motor in units of encoder counts.
        @param Kp A control gain set by the user. This is used to produce the actuation signal for the motor.
        @param Kd a controller gain set by the user. This is used for the controller's Kd implementation. 
        @param read_fun This is a read function from an Encoder Object.

        """
        self.Kp = Kp
        self.setpoint = setpoint
        self.output_fun = read_fun
        self.pos_output = []
        self.Kd = Kd

        # values for Kd control
        # timer values used for calculating rate of change of error
        #self.prev_time = 0  # previous time looping
        #self.curr_time = 0  # current time looping
        
        self.T = 0   # this is the trigger rate for the controller
        self.delta_err = 0 # this store the change in error for derivative control, the first value should be error
        self.prev_err = 0 # previous error value for delta_err calculation

        self.err = 51 # for initilization, need an initial error for the control loop
        self.store = True    # for storing values, store values if self.True == True
        
    def run(self, meas_output):
        """!
        This function runs the controller and changes the PWM for the next cycle. 
        The control loop takes the input encoder tics from meas_output and runs it through the control loop.
        This control loop includes proportional and derivative control, meaning the Kp gain is multiplied by the current error
        and the Kd values is multiplied by the change in error. The products are then added together and the resulting value
        is the PWM sent to the motor. 
    
        @param meas_output This value is the previous measured output input to the controller function. 
                           The value of meas_output is in encoder tics and tracks the current position of the motor. 
        """
        # create the equation that runs the control loop
        # calculate error
        self.err = self.setpoint - meas_output

        # calculate delta error for Kd
        self.delta_err = self.err - self.prev_err

        # calculate the change in time, for Kd
        #self.curr_time = utime.time()
        #self.T = self.curr_time - self.prev_time
        self.T = 0.01 # rough estimate, in ms, of the board's trigger rate

        # calculate PWM
        # the equation below represents the control loop to calculate the required motor PWM 
        self.PWM = self.Kp*(self.err) + (self.Kd*(self.delta_err/self.T))    

        # store values for Kd calculation
        self.prev_err = self.err
        #self.prev_time = utime.time()

        # store values for graphing
        if self.store == True:  # if self.store == True store values for step-response graphing
            self.pos_output.append(meas_output)
        if len(self.pos_output) >= 1000:    # need to stop storing values if there are too many
            self.store = False  # set store to false
        
    
    def set_setpoint(self,setpoint):
        """!
        This function sets the setpoint for the controller object. The setpoint is in encoder tics and used in the control loop to 
        calculate error. 
    
        @param setpoint This is a location for the motor in units of encoder counts.
        """
        self.setpoint=setpoint  # store in the class object
        
    def set_Kp(self,Kp):
        """!
        This function sets the Kp value for the controller object. The Kp is the proportional gain used in the control loop. 
    
        @param Kp A control gain set by the user. This is used to produce the actuation signal.
        """
        self.Kp=Kp

    def set_Kd(self,Kd):
        """!
        This function sets the Kd value for the controller object. The Kd value is the derivative gain used in the control loop. 
    
        @param Kd A control gain set by the user. This is used to produce the actuation signal.
        """
        self.Kd=Kd
        
    def step_response(self):
        '''!
        This function allows the stored values from the control loop to be plotted for a step-response graph. 
        This function was used in the main.py file along with our gui code to plot different step-responses for testing. 
        '''
        print('start')  # print 'start' to tell the computer to be ready for values
        for i in range(len(self.pos_output)):
            print(i*3.333,self.pos_output[i]) # print each output value along with its time value, does a step about every 3 miliseconds
        print('end')    # print 'end' to tell the computer the values are ended
        
    def reset_loop(self):
        """!
        This function resets the calculations for a new control loop. 
        This function can be used in conjuction with encoderX.read() & set_setpoint() for a new control loop of a different angle

        """
        # reset variables for the Kd calculation
        self.delta_err = 0 # this store the change in error for derivative control, the first value should be error
        self.prev_err = 0 # previous error value for delta_err calculation

        # reset the timer
        #self.prev_time = 0  # previous time looping
        #self.curr_time = 0  # current time looping
       


    
    