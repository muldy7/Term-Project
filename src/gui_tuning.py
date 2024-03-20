"""!
@file gui_tuning.py
This file contains code to run program on a laptop or desktop which creates a user interface
that can send a signal to the microcontroller to run a step response. The user can set a Kp, Kd, and Setpoint
values and send that to a microcontroller where a Controller Object will read and interpret the data
from the motor. This code was used to tune our robot for different step responses during its use. Using the code below 
we found the best Kp and Kd values for the 180 degree turn, which we found to be different than the optimal gain values
for smaller turns that would occur while finding a target. This testing and gain values are documented in our README file, and where implemented
in our main project files. 

This file is based on the GUI file from Lab 3, but with extra text entries for Kd and Setpoint. Implementation of the code is documented below.

The code used in main.py on our microcontroller can be found in the main.py file in our Doxygen and GitHub documentation. 

This file is uses code from lab0example.py file on on Cantvas and an example found at:
https://matplotlib.org/stable/gallery/user_interfaces/embedding_in_tk_sgskip.html

@author Abe Muldrow
@author Lucas Rambo
@author Peter Tomson
@date March 2nd, 2024, Original program, based on example from above listed sources
"""

# imports 
import math
import time
import tkinter
import serial
from random import random
import random as random
from serial import Serial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
    

def step_response(plot_axes, plot_canvas, xlabel, ylabel, entry, entry2, entry3):	
    """!
    This function retrieves the data from the microcontroller to create the plot of the step response.
    The function first sens ASCII digits to the controller to stop any running program, get the controller into
    regular REPL mode, and then reboot the code to cause the main.py code on the board to run. The main.py file
    contains the code to generate the step response which is then read by this function through the serial port. 
    
    @param plot_axes The plot axes supplied by Matplotlib
    @param plot_canvas The plot canvas, also supplied by Matplotlib
    @param xlabel The label for the plot's horizontal axis
    @param ylabel The label for the plot's vertical axis
    @param entry text box that contains the Kp value from the user
    @param entry2 text box that contains the Kd value from the user
    @param entry3 text box that contains the setpoint value from the user
    """
    # set the serial port for reading from the microcontroller
    ser = serial.Serial("COM3", 9600)
    
    # reset and send commands to the board in ASCII
    print("waiting for data")
    ser.write(bytearray('\x03','ascii')) # ascii code for ctrl+c
    ser.write(bytearray('\x02','ascii')) # ctrl+b
    ser.write(bytearray('\x04','ascii')) # ctrl+d
    print("sending to board")
    
    # create our variables for printing 
    array =[]	# create an array to store the data
    cont = 1	# variable for running the read function
    time=[]	 	# list to store our time values
    pos=[]	# list to store our volt values
    start=0 #b oolean to show start of step response data
    enter = 0
    # while cont is true read values from the serial port
    
    # using entry.get() will retrieve the current number in the entry box
    # retrieve the gain and setpoint values
    kp = entry.get()
    kd = entry2.get()	
    set_point = entry3.get()

    try:
        float(kp)
    except ValueError:  # test if the user doesn't enter a valid number
        print('Please Enter a Number')
        cont = 0
        return
    
    # loop to read step response values from the board
    while cont == 1:
        value = ser.readline().decode('utf-8').strip()  # read from the nucleo board
        if value == 'awaiting input' and enter == 0:    # wait till the board prints 'awaiting input'
            # after awaiting input, the main.py code on the board has three different value() text entries
            # the three entries in the main.py code change the values based on values from the user 
            ser.write(bytes(str(kp).encode('utf-8')))	# this is where kp is used, sent to the input function waiting on the board
            ser.write(bytearray('\x0D','ascii'))    # send an enter to the board
            ser.write(bytes(str(kd).encode('utf-8')))	# this is where kd is used, sent to the input function waiting on the board
            ser.write(bytearray('\x0D','ascii'))    # send an enter to the board
            ser.write(bytes(str(set_point).encode('utf-8')))	# this is where set_point is used, sent to the input function waiting on the board
            ser.write(bytearray('\x0D','ascii'))    # send an enter to the board
            print('starting a step response')
            enter = 1
            
        if value == 'start':    # start string signals the start of step-response values from the board
            start=1
            print("reading!")
        while start==1:
            value = ser.readline().decode('utf-8').strip()  # read values
            print(value)
            if value == 'end':	# once end is printed by the microcontroller stop reading values
                start = 0   # change the booleans
                cont=0
                break
            else:
                array.append(value)	# append values to out array for graphing of our step-response
                    
    for i in array:	# once we have retrieved the values from the board they must be added to our lists
        index = i.split(' ')	# split x and y values
        try:  
            timeval  =float(index[0].strip('('))	# float and strip the value
            posval = float(index[1].strip(')'))
            time.append(timeval)	# add to out lists
            pos.append(posval)
        except:
            pass
   
    print('plotting data')
    
    # add a list colors for graphing
    colors = ['deepskyblue', 'firebrick','springgreen', 'wheat', 'fuchsia', 'olivedrab', 'linen', 'salmon']
    color1 = random.choice(colors)  # create a random color choice

    # plot the step response
    # add the Kp, Kd, and Setpoint values to the plot
    plot_axes.plot(time, pos, label = 'Measured Response Data, kp: '+ kp + ' kd: ' + kd + ' sp: ' + set_point, color = color1, marker = '.' )   
    plot_axes.set_xlabel(xlabel)
    plot_axes.set_ylabel(ylabel)
    plot_axes.grid(True)
    plot_axes.legend()
    plot_axes.axis([0, 300, 0, 7000])	# change the axis based on how many encoder tics we are graphing
    plot_canvas.draw()
    


def tk_matplot(plot_function, xlabel, ylabel, title):
    """!    
    Create a TK window with one embedded Matplotlib plot.
    This function makes the window, displays it, and runs the user interface
    until the user closes the window. The plot function, which must have been
    supplied by the user, should draw the plot on the supplied plot axes and
    call the draw() function belonging to the plot canvas to show the plot. 
    @param plot_function The function which, when run, creates a plot
    @param xlabel The label for the plot's horizontal axis
    @param ylabel The label for the plot's vertical axis
    @param title A title for the plot; it shows up in window title bar
    """
    # Create the main program window and give it a title
    tk_root = tkinter.Tk()
    tk_root.wm_title(title)

    # Create a Matplotlib 
    fig = Figure()
    axes = fig.add_subplot()

    # Create the drawing canvas and a handy plot navigation toolbar
    canvas = FigureCanvasTkAgg(fig, master=tk_root)
    toolbar = NavigationToolbar2Tk(canvas, tk_root, pack_toolbar=False)
    toolbar.update()

    # Create the buttons that run tests, clear the screen, and exit the program
    entry = tkinter.Entry(master=tk_root, textvariable='')	# this creates the entry box for Kp
    
    # entry for Kd
    entry2 = tkinter.Entry(master=tk_root, textvariable='')	# this creates the entry box
    
    # entry for setpoint
    entry3 = tkinter.Entry(master=tk_root, textvariable='')
    
    # the eneter button takes the kp value and sends it to run
    button_enter = tkinter.Button(master=tk_root,
                                  text="Run Step Response", command=lambda: plot_function(axes, canvas,
                                                              xlabel, ylabel, entry, entry2, entry3))
   
    button_quit = tkinter.Button(master=tk_root,
                                 text="Quit",
                                 command=tk_root.destroy)
    button_clear = tkinter.Button(master=tk_root,
                                  text="Clear",
                                  command=lambda: axes.clear() or canvas.draw())

    # Arrange things in a grid because "pack" is weird
    canvas.get_tk_widget().grid(row=0, column=0, columnspan=4)
    toolbar.grid(row=1, column=0, columnspan=4)
    
    # buttons
    button_clear.grid(row=2, column=3)
    button_quit.grid(row=4, column=3)
    entry.grid(row = 2, column = 1)
    entry2.grid(row = 3, column = 1)
    entry3.grid(row = 4, column = 1)
    button_enter.grid(row=2, column = 2)

    # labels
    tkinter.Label(master=tk_root, text="Enter Kp Value:").grid(row=2,column=0)
    tkinter.Label(master=tk_root, text="Enter Kd Value:").grid(row=3,column=0)
    tkinter.Label(master=tk_root, text="Enter Desired Setpoint:").grid(row=4,column=0)

    # This function runs the program until the user decides to quit
    tkinter.mainloop()
    
    
    
# This main code is run if this file is the main program but won't run if this
# file is imported as a module by some other main program
if __name__ == "__main__":
    #run the tk_matplot function with the proper plot function and labels
    tk_matplot(step_response,
               xlabel='Time [ms]',
               ylabel='Encoder Position [ticks]',
               title='Kp Sensitive Step Response')
        
