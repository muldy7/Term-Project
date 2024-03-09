'''!
@file controller.py

This file contains code to create a controller class that allows us to create a control loop to run our motor.
This controller class when used in conjuction with our motor_driver and encoder_reader classes allow use to implement 
closed loop porportional control of our motor. The class below includes function to init and run out controller, 
along with setting the setpoint and Kp value of the control loop. 

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
'''



class Controller:
    
    def __init__ (self, setpoint, Kp, read_fun):
        """! 
        Creates a Controller Object by initializing a setpoint (a desired location for the motor)
        a Kp value given by a user, and a encoder read function from an Encoder Object.

        @param setpoint This is a location for the motor in units of encoder counts.
        @param Kp A control gain set by the user. This is used to produce the actuation signal.
        @param read_fun This is a read function from an Encoder Object.
        """
        self.Kp=Kp
        self.setpoint=setpoint
        self.output_fun=read_fun
        self.pos_output=[]
        
    def run(self,meas_output):
        """!
        This function runs the controller and changes the PWM for the next cycle.
    
        @param meas_output This value is the previous measured output.
        """
        # create the equation that runs the control loop
        self.PWM=self.Kp*(self.setpoint-meas_output)
        self.pos_output.append(meas_output)
    
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
        
    def step_response(self):
        print('start')
        for i in range(len(self.pos_output)):
            print(i*10,self.pos_output[i])
        print('end')
        
    
    