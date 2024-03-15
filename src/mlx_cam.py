## @file mlx_cam.py
# 
#  RAW VERSION
#  This version uses a stripped down MLX90640 driver which produces only raw
#  data, not calibrated data, in order to save memory.
# 
#  This file contains a wrapper that facilitates the use of a Melexis MLX90640
#  thermal infrared camera for general use. The wrapper contains a class MLX_Cam
#  whose use is greatly simplified in comparison to that of the base class,
#  @c class @c MLX90640, by mwerezak, who has a cool fox avatar, at
#  @c https://github.com/mwerezak/micropython-mlx90640
# 
#  To use this code, upload the directory @c mlx90640 from mwerezak with all
#  its contents to the root directory of your MicroPython device, then copy
#  this file to the root directory of the MicroPython device.
# 
#  There's some test code at the bottom of this file which serves as a
#  beginning example.
# 
#  @author mwerezak Original files, Summer 2022
#  @author JR Ridgely Added simplified wrapper class @c MLX_Cam, January 2023
#  @copyright (c) 2022-2023 by the authors and released under the GNU Public
#      License, version 3.

import utime as time
#import csv
from machine import Pin, I2C
from mlx90640 import MLX90640
from mlx90640.calibration import NUM_ROWS, NUM_COLS, IMAGE_SIZE, TEMP_K
from mlx90640.image import ChessPattern, InterleavedPattern



## @brief   Class which wraps an MLX90640 thermal infrared camera driver to
#           make it easier to grab and use an image. 
#  @details This image is in "raw" mode, meaning it has not been calibrated
#           (which takes lots of time and memory) and only gives relative IR
#           emission seen by pixels, not estimates of the temperatures.
class MLX_Cam:

    ## @brief   Set up an MLX90640 camera.
    #  @param   i2c An I2C bus which has been set up to talk to the camera;
    #           this must be a bus object which has already been set up
    #  @param   address The address of the camera on the I2C bus (default 0x33)
    #  @param   pattern The way frames are interleaved, as we read only half
    #           the pixels at a time (default ChessPattern)
    #  @param   width The width of the image in pixels; leave it at default
    #  @param   height The height of the image in pixels; leave it at default
    def __init__(self, i2c, address=0x33, pattern=ChessPattern,
                 width=NUM_COLS, height=NUM_ROWS):

        ## The I2C bus to which the camera is attached
        self._i2c = i2c
        ## The address of the camera on the I2C bus
        self._addr = address
        ## The pattern for reading the camera, usually ChessPattern
        self._pattern = pattern
        ## The width of the image in pixels, which should be 32
        self._width = width
        ## The height of the image in pixels, which should be 24
        self._height = height
        ## Tracks whether an image is currently being retrieved
        self._getting_image = False
        ## Which subpage (checkerboard half) of the image is being retrieved
        self._subpage = 0

        # The MLX90640 object that does the work
        self._camera = MLX90640(i2c, address)
        self._camera.set_pattern(pattern)
        self._camera.setup()

        ## A local reference to the image object within the camera driver
        self._image = self._camera.raw


    ## @brief   Show low-resolution camera data as shaded pixels on a text
    #           screen.
    #  @details The data is printed as a set of characters in columns for the
    #           number of rows in the camera's image size. This function is
    #           intended for testing an MLX90640 thermal infrared sensor.
    #  
    #           A pair of extended ACSII filled rectangles is used by default
    #           to show each pixel so that the aspect ratio of the display on
    #           screens isn't too smushed. Each pixel is colored using ANSI
    #           terminal escape codes which work in only some programs such as
    #           PuTTY.  If shown in simpler terminal programs such as the one
    #           used in Thonny, the display just shows a bunch of pixel
    #           symbols with no difference in shading (boring).
    # 
    #           A simple auto-brightness scaling is done, setting the lowest
    #           brightness of a filled block to 0 and the highest to 255. If
    #           there are bad pixels, this can reduce contrast in the rest of
    #           the image.
    # 
    #           After the printing is done, character color is reset to a
    #           default of medium-brightness green, or something else if
    #           chosen.
    #  @param   array An array of (self._width * self._height) pixel values
    #  @param   pixel Text which is shown for each pixel, default being a pair
    #           of extended-ASCII blocks (code 219)
    #  @param   textcolor The color to which printed text is reset when the
    #           image has been finished, as a string "<r>;<g>;<b>" with each
    #           letter representing the intensity of red, green, and blue from
    #           0 to 255
    def ascii_image(self, array, pixel="██", textcolor="0;180;0"):

        minny = min(array)
        scale = 255.0 / (max(array) - minny)
        for row in range(self._height):
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                           - minny) * scale)
                print(f"\033[38;2;{pix};{pix};{pix}m{pixel}", end='')
            print(f"\033[38;2;{textcolor}m")


    ## A "standard" set of characters of different densities to make ASCII art
    asc = " .@"


    ## @brief   Show a data array from the IR image as ASCII art.
    #  @details Each character is repeated twice so the image isn't squished
    #           laterally. A code of "><" indicates an error, probably caused
    #           by a bad pixel in the camera. 
    #  @param   array The array to be shown, probably @c image.v_ir
    def ascii_art(self, array):

        scale = len(MLX_Cam.asc) / (max(array) - min(array))
        offset = -min(array)
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                           + offset) * scale)
                try:
                    the_char = MLX_Cam.asc[pix]
                    print(f"{the_char}{the_char}", end='')
                except IndexError:
                    print("><", end='')
            print('')
        return


    ## @brief   Generate a string containing image data in CSV format.
    #  @details This function generates a set of lines, each having one row of
    #           image data in Comma Separated Variable format. The lines can
    #           be printed or saved to a file using a @c for loop.
    #  @param   array The array of data to be presented
    #  @param   limits A 2-iterable containing the maximum and minimum values
    #           to which the data should be scaled, or @c None for no scaling
    def get_csv(self, array, limits=None):

        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        for row in range(self._height):
            line = ""
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                if col:
                    line += ","
                line += f"{pix}"
            yield line
        return


    ## @brief   Get one image from a MLX90640 camera, @b blocking other tasks
    #           from running until the image has been received.
    #  @details Grab one image from the given camera and return it. Both
    #           subframes (the odd checkerboard portions of the image) are
    #           grabbed and combined (maybe; this is the raw version, so the
    #           combination is sketchy and not fully tested). It is assumed
    #           that the camera is in the ChessPattern (default) mode as it
    #           probably should be.
    #  @returns A reference to the image object we've just filled with data
    def get_image(self):

        for subpage in (0, 1):
            while not self._camera.has_data:
                time.sleep_ms(50)
            image = self._camera.read_image(subpage)

        return image


    ## @brief   Get an image from an MLX90640 camera in a non-blocking way.
    #  @details This function is to be called repeatedly; it will return @c None
    #           until a complete image has been retrieved (this takes around a
    #           quarter to half second) and will then return the image.
    #
    #      @b Example: This code would be inside a task function which yields 
    #      repeatedly as long as there isn't a complete image available.
    #      @code
    #      image = None
    #      while not image:
    #          image = camera.get_image_nonblocking()
    #          yield(state)
    #      @endcode
    #
    def get_image_nonblocking(self):

        # If this is the first recent call, begin the process
        if not self._getting_image:
            self._subpage = 0
            self._getting_image = True
        
        # Read whichever subpage needs to be read, or wait until data is ready
        if not self._camera.has_data:
            return None
        
        image = self._camera.read_image(self._subpage)
        
        # If we just got subpage zero, we need to come back and get subpage 1;
        # if we just got subpage 1, we're done
        if self._subpage == 0:
            self._subpage = 1
            return None
        else:
            self._getting_image = False
            return image
       
    # abe is going to make this to try and get the total value from the camera, instead of going through the CSV
    def get_hotspot(self, array, centroid, limits=None):

        if limits and len(limits) == 2:
            scale = (limits[1] - limits[0]) / (max(array) - min(array))
            offset = limits[0] - min(array)
        else:
            offset = 0.0
            scale = 1.0
        
        # create a 2D array out of lists that is the size of the image_arr
        rows, cols = (self._height, self._width)    # for array creation
        image_arr = [[' ' for i in range(cols)] for j in range(rows)]	# create an image array
        
        # add the pixel value from the image to the image 2D list to store values for calculations
        for row in range(self._height):
            for col in range(self._width):
                pix = int((array[row * self._width + (self._width - col - 1)]
                          + offset) * scale)
                
                image_arr[row][col] = pix	# add each pixel to the array
        
        # sum the colums
        self.sums = []	# create the list
        
        # iterate through each row and then each column
        for i in range(self._width):	 # go through each column, i
            total = 0	# create a total for each column
            for j in range(self._height): # go through each row, k
                if j >= 17:	# set a vertical limit to ignore the bottom rows, can limit a top portion of the rows as well if thats helpful 
                    continue
                elif float(image_arr[j][i]) <= 250: # eliminate noise from the camera image, all values less than 200
                    continue
                else:
                    total += float(image_arr[j][i])	# add the current value from image_arr to the total 
            self.sums.append(total)	# add the value of each column to sums
            
       # can use enumerate to iterate through the list and record the index and max value of the location it is at
        self.max_value = 0	# create max index and value
        self.max_index = 0
       
        # enumerate through self.sums to test for the larget value
        for idx,val in enumerate(self.sums):
            if idx <= 10 or idx >= 20:	# ignore values too far to the left or right 	# horizontal limit
                continue
            if val > self.max_value:	# find the largest value in the list
               self.max_index = idx	# store the index of the biggest value, this gives us degree location of our target
               self.max_value = val	# store the biggest value
               
               # print for testing
               #print(self.max_index)
        
        
        # values for centroid calculation
        i_l = self.max_index-1	# index to the left of the device
        i_r = self.max_index+1	# index to the right of the device
        
        # ignore indexes if too far to right or left
        if self.max_index == 0:
            i_l = 0
        if self.max_index == 31:
            i_r = 31

        # test to see which adjacent index value is bigger for centroid calculation
#         if self.sums[i_l] >= self.sums[i_r]:
#             i_calc = i_l # store the idex value for calculation
#         else:
#             i_calc = i_r
        
        # encoder tic/camera column constant
        # this value is used for the encoder tics calculation
        k_degree = 62.64 # this is the amount of encoder ticks per degree of MLX cam
                         # the MLX cam is 55 degrees wide, width of 32, 1.72 degrees per width, then 6600 is one full rotation of our bot
                       
        # do the centroid calculation if centroid = True
        if centroid == True:	# calc centroid if centroid equals true
            
                
            # sum the indexes multiplied by their avg value
            self.i_bar = ((self.max_index*(self.max_value)) + ((i_l)*(self.sums[i_l]))) + (i_r)*(self.sums[i_r]) + (i_r + 1)*(self.sums[i_r + 1]) + (i_l - 1)*(self.sums[i_l - 1])
            
            # divide by the sums of their average values
            self.i_bar = self.i_bar/(self.max_value + self.sums[i_r] + self.sums[i_l] + self.sums[i_r + 1] + self.sums[i_l - 1])
           
            # print values for testing
            print('ibar: ' + str(self.i_bar))
            print('i_max: ' + str(self.max_index))
            print(self.sums)
            
            # calculate camera_error
            # 15.5 is the middle of the camera, multiply by the K_degree constant
            self.camera_error = (-15.5+self.i_bar)*k_degree
            
        else:	# calc with just i_max if centroid = False
            self.camera_error = (-15.5+self.max_index)*k_degree
        
    
## This test function sets up the sensor, then grabs and shows an image in a
#  terminal every few seconds. By default it shows ASCII art, but it can be
#  set to show better looking grayscale images in some terminal programs such
#  as PuTTY. Unfortunately Thonny's terminal won't show the nice grayscale. 
def test_MLX_cam():

    import gc

    # The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
    try:
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
    
    print(camera._height)
    time.sleep(5)
    
    while True:
        try:
            # Get and image and see how long it takes to grab that image
            print("Click.", end='')
            begintime = time.ticks_ms()
#             image = camera.get_image()

            # Keep trying to get an image; this could be done in a task, with
            # the task yielding repeatedly until an image is available
            image = None
            while not image:
                image = camera.get_image_nonblocking()
                time.sleep_ms(5) #50

            print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

            # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
            # Display pixellated grayscale or numbers in CSV format; the CSV
            # could also be written to a file. Spreadsheets, Matlab(tm), or
            # CPython can read CSV and make a decent false-color heat plot.
            show_image = False
            show_csv = False
            if show_image:
                camera.ascii_image(image)
            elif show_csv:
                for line in camera.get_csv(image, limits=(0, 99)):	# this is mlx cam
                    print(line)
            else:
                camera.ascii_art(image)
                time.sleep(5)
            gc.collect()
            print(f"Memory: {gc.mem_free()} B free")
            time.sleep_ms(5) #3141

        except KeyboardInterrupt:
            break

    print ("Done.")
    
# abe added this function to try and average the camera data 
def camera_data():

    import gc

    # The following import is only used to check if we have an STM32 board such
    # as a Pyboard or Nucleo; if not, use a different library
    try:
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

    while True:
        try:
            # Get and image and see how long it takes to grab that image
            print("Click.", end='')
            begintime = time.ticks_ms()
#             image = camera.get_image()

            # Keep trying to get an image; this could be done in a task, with
            # the task yielding repeatedly until an image is available
            image = None
            while not image:
                image = camera.get_image_nonblocking()
                time.sleep_ms(5) #50

            print(f" {time.ticks_diff(time.ticks_ms(), begintime)} ms")

            # Can show image.v_ir, image.alpha, or image.buf; image.v_ir best?
            # Display pixellated grayscale or numbers in CSV format; the CSV
            # could also be written to a file. Spreadsheets, Matlab(tm), or
            # CPython can read CSV and make a decent false-color heat plot.
            show_image = False
            show_csv = True
            total = 0
            if show_image:
                camera.ascii_image(image)
            elif show_csv:
                camera.get_hotspot(image, True, limits=(0, 1000))
                    
                        
            else:
                camera.ascii_art(image)
            gc.collect()
            print(f"Memory: {gc.mem_free()} B free")
            time.sleep_ms(3141) #3141

        except KeyboardInterrupt:
            break

    print ("Done.")        

if __name__ == "__main__":
    
    
    camera_data()
    print(camera.avg)
    print(camera.i_bar)
    utime.sleep(5)
    
#     test_MLX_cam()
#     print(camera._height)


