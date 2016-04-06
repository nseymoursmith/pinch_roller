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
import picamera
from SimpleCV import Image, Color
from SimpleCV.DrawingLayer import DrawingLayer
import subprocess


#------PID settings -------
Kp = 0.1
Ki = 0.1
Kd = 0.001
set_point = 262
error = 0
error_last = 0
out_max = 255
out_min = 0
output_offset = 125 #just until we write a mode for pulling through at start
forward = True
P = 0.0
I = 0.0
D = 0.0
output = 0
#Ought to use a real time clock if going to do this onboard (i.e. not arduino)
t = time.time()
last_time = t
dt = 0.0
output_rate = 1.0 #capture/output rate
last_out = time.time()

combine = lambda pid, offset, forward: (offset + pid) if forward else (offset - pid)
clamp = lambda n, n_min, n_max: max(min(n_max, n), n_min)
#--------------------------

bg = False #true for black background, false for white
ARDUINO = False #False if testing microscope without arduino

#Functions for mapping image to filament width
pixel_threshold = 200

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
def getWidth(xSection, pixel_threshold, bg):
    inside_pixels = 0
    upper_edge = 0
    lower_edge = 0
    inside = False
    comp = lambda x, y, bg: (x > y) if bg else (x < y)
    for i, p in enumerate(xSection):
        if comp(p, pixel_threshold, bg):
            inside_pixels += 1
            if not inside:
                upper_edge = i
                inside = True
        else:
            if inside:
                lower_edge = i
                inside = False

    return (inside_pixels, upper_edge, lower_edge)

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
prop_map = {
#    "width": 640,
#    "height": 480,
    "brightness": 0,
    "contrast": 1,
    "gain": 0,
    "hue": 0,
    "saturation": 0,
#            "exposure": 1, #exposure not supported for this camera/system
}

#cam = Camera(-1,prop_map)
#print cam.getAllProperties()
cam = picamera.PiCamera()

#cam = Camera()
logger = logging.getLogger(__name__)
if ARDUINO:
    port = "/dev/tty*ACM*"
    try:
        port = glob.glob(port)[0]
    except IndexError:
        logger.error("Could not find serial port for pinch roller arduino: %s" % (port,))
        raise Exception(port + " does not exist!")
    try:
        connection = serial.Serial(port, 115200, timeout=None) # serial port which blocks forever on read
    except:
        logger.exception("Error opening serial port for pinch roller arduino: %s" % (port,))
        raise
    logger.debug("Serial initialised to port: %s \n" % (port,))

# The main loop
while True:
    try:
#	subprocess.call("raspistill -n -t 0 -w %s -h %s -o image.bmp" % (640, 480), shell=True)
	cam.capture('image.jpg')
        img = Image("image.jpg")
        img.show()
        xSection = getXSection(img)
        width_data = getWidth(xSection, pixel_threshold, bg)
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

        img.show()

        #PID calculations
        error = width - set_point
        P = Kp*error
        t = time.time()
        dt = t - last_time
        last_time = t
        I += Ki*error*dt
        I = clamp(I, -out_max, out_max)
        D = Kd*(error - error_last)/dt
        error_last = error
        output = combine(int(P + I + D), output_offset, forward)
        output = clamp(output, out_min, out_max)

        if ARDUINO:
            if (t - last_out) > output_rate:
                sendMessage(connection, str(output))
                last_out = t

        print "width: " + str(width)
        print "set point: " + str(set_point)
        print "error: " + str(error)
        print "P: %d, I: %d, D: %d" % (int(P) , int(I) , int(D))
        print "output: " + str(output)
        print "-------------"
    
       
    except (KeyboardInterrupt, SystemExit):
	if ARDUINO:
  		sendMessage(connection, str(0))
        	connection.close()
        raise
