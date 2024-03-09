'''!
@file pd_controller.py

This file attempts to implement pd control of our device with some small changes to the previous controller class

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


class Controller:
    
    def __init__ (self, setpoint, Kp, Kd,read_fun):
        """! 
        Creates a Controller Object by initializing a setpoint (a desired location for the motor)
        a Kp value given by a user, and a encoder read function from an Encoder Object.

        @param setpoint This is a location for the motor in units of encoder counts.
        @param Kp A control gain set by the user. This is used to produce the actuation signal.
        @param read_fun This is a read function from an Encoder Object.

        @param Kd this is the derivative gain for our controller
        """
        self.Kp=Kp
        self.setpoint=setpoint
        self.output_fun=read_fun
        self.pos_output=[]
        self.Kd = Kd
        self.T = 0.01   # this is the trigger rate for the controller, going to assume like 1 ms but we can test
        self.delta_err = 0 # this store the change in error for derivative control, the first value should be error
        self.err = 0
        self.prev_err = 0 # previous error value for delta_err calculation
        
    def run(self,meas_output):
        """!
        This function runs the controller and changes the PWM for the next cycle.
    
        @param meas_output This value is the previous measured output.
        """
        # create the equation that runs the control loop
        self.err = self.setpoint - meas_output
        self.delta_err = self.err - self.prev_err
        self.PWM=self.Kp*(self.err) + self.Kd*(self.delta_err/self.T)    # it should technically be possible to add a integral controller to this as well but I dont know what the intergal of position is 
        self.pos_output.append(meas_output)
        self.prev_err = self.err
    
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
        
    def step_response(self):
        print('start')
        for i in range(len(self.pos_output)):
            print(i*10,self.pos_output[i])
        print('end')


    # add reset loop here
        
    
    