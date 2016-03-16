#!/usr/bin/python
'''
N. Seymour-Smith 10/3/16 for Noztek
Filament pinch roller control. Monitors width of filament on microscope camera,
feeds back to arduino controlling pinch roller motor
'''
import serial
import glob
import time
import logging
from SimpleCV import Image, Camera, Color
from SimpleCV.DrawingLayer import DrawingLayer

#------PID settings -------
Kp = 1.0
Ki = 0.0
Kd = 0.0
set_point = 220
error = 0
error_last = 0
out_max = 255
out_min = 0
clamp = lambda n, n_min, n_max: max(min(n_max, n), n_min)
output_offset = 127
P = 0.0
I = 0.0
D = 0.0
output = 0
#Ought to use a real time clock if going to do this onboard (i.e. not arduino)
t = time.time()
last_time = t
dt = 0.0
#--------------------------

#Functions for mapping image to filament width
pixel_threshold = 400

#Can replace with a more complex function of img if necessary
def getXSection(img):
    width = img.width
    height = img.height
    xSection = []
    for y in range(height):
        current_pixel = img.getPixel(width/2,y)
        xSection.append(sum(current_pixel))
    return xSection

#Can replace with a more complex function of x-section if necessary
def getWidth(xSection, pixel_threshold):
    dark_pixels = 0
    upper_edge = 0
    lower_edge = 0
    dark = False
    for i, p in enumerate(xSection):
        if p < pixel_threshold:
            dark_pixels += 1
            if not dark:
                upper_edge = i
                dark = True
        else:
            if dark:
                lower_edge = i
                dark = False
    return (dark_pixels, upper_edge, lower_edge)

#Serial functions
def sendMessage(serial_connection, message):
    #Flush Serial buffer
    while serial_connection.inWaiting():
        flush_lines = serial_connection.readline()
    serial_connection.write(message)

def get_message(serial_connection, timeout = 10): #timeout in seconds
    while serial_connection.inWaiting():
        flush_lines = serial_connection.readline()
    start_time = time.time()
    while not serial_connection.inWaiting(): #Wait for message from arduino
        time.sleep(0.1)
        if time.time() - start_time > timeout:
            logger.error("Serial communication timed out")
            raise Exception("Serial timeout")
    received_line = serial_connection.readline() #receive inbound lines
    return received_line

# Set up camera and serial connection
cam = Camera()
logger = logging.getLogger(__name__)
port = "/dev/tty*USB*"
try:
    port = glob.glob(port)[0]
except IndexError:
    logger.error("Could not find serial port for pinch roller arduino: %s" % (port,))
    raise Exception(port + " does not exist!")
try:
    connection = serial.Serial(port, 9600, timeout=None) # serial port which blocks forever on read
except:
    logger.exception("Error opening serial port for pinch roller arduino: %s" % (port,))
    raise
logger.debug("Serial initialised to port: %s \n" % (port,))

# The main loop
while True:
    img = cam.getImage()
    xSection = getXSection(img)
    width_data = getWidth(xSection, pixel_threshold)
    width = width_data[0]

    upper_edge = width_data[1]
    lower_edge = width_data[2]

    upper_line = DrawingLayer((img.width, img.height))
    lower_line = DrawingLayer((img.width, img.height))
    x_mid_line = DrawingLayer((img.width, img.height))

    upper_line.line((0,upper_edge), (img.width, upper_edge), alpha=128, width=1, color=Color.RED)
    lower_line.line((0,lower_edge), (img.width, lower_edge), alpha=128, width=1, color=Color.RED)
    x_mid_line.line((img.width/2,0), (img.width/2, img.height), alpha=128, width=1, color=Color.RED)

    img.addDrawingLayer(upper_line)
    img.addDrawingLayer(lower_line)
    img.addDrawingLayer(x_mid_line)

    print "width: " + str(width)

    #PID calculations
    error = width - set_point
    P = Kp*error
    t = time.time()
    dt = t - last_time
    last_time = t
    I += Ki*error*dt
    D = Kd*(error - error_last)/dt
    output = output_offset - int(P + I + D)
    output = clamp(output, out_min, out_max)
    print "output: " + str(output)
    print "-------------"

    sendMessage(connection, str(output))
    img.show()
