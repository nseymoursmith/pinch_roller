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
import io
from numpy import polyfit
from SimpleCV import Image, Camera, Color
from SimpleCV.DrawingLayer import DrawingLayer


#General settings
PI_CAM = True #If using raspberry picam
ARDUINO = False #False if testing microscope without arduino
pixel_threshold = 30

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


#Functions for mapping image to filament width

# Make a function that does a half and half image.
def halfsies(left,right): 
    result = left
    # crop the right image to be just the right side.
    crop   = right.crop(right.width/2.0,0,right.width/2.0,right.height)
    # now paste the crop on the left image.
    result = result.blit(crop,(left.width/2,0))
    # return the results.
    return result

#Can replace with a more complex function of img if necessary
def getXSection(width, height, x_posn):
    xSection = []
    for y in range(height):
        current_pixel = img.getPixel(x_posn,y)
        xSection.append(sum(current_pixel))
    return xSection

def getEdges(xSection):
    edges = []
    for i, p in enumerate(xSection):
        if p:
            edges.append(i)
    try:
        return(min(edges), max(edges))
    except:
        return(0,0)

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
    "brightness": 0.1,
    "contrast": 0.9,
    "gain": 0.1,
    "hue": 0,
    "saturation": 0,
#            "exposure": 1, #exposure not supported for this camera/system
}

if PI_CAM:
    import picamera
    from picamera import array as parray
    cam = picamera.PiCamera()
    cam.resolution = (640,480)
else:
    cam = Camera(-1,prop_map)

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
        width = 0
        if PI_CAM:
#            cam.capture(stream,'jpeg', use_video_port=True)
	    with parray.PiRGBArray(cam) as output:
                cam.capture(output, 'rgb', use_video_port=True)
                img = Image(output.array)
        else:
            img = cam.getImage()

        #edge detect
        output = img.edges(t1=pixel_threshold, t2=4*pixel_threshold)

        #split screen
        result = halfsies(img,output)

        #find the edges
        upper_edge = []
        lower_edge = []
        
        for x in range(0,img.width):
            xSection = output.getVertScanlineGray(x)
            edge_data = getEdges(xSection)
            
            if edge_data[0] and edge_data[1]:
                upper_edge.append(edge_data[0])
                lower_edge.append(edge_data[1])

        #fit the edge data
        try:
            upper_line = polyfit(range(0,len(upper_edge)), upper_edge, 1)
            lower_line = polyfit(range(0,len(lower_edge)), lower_edge, 1)

            #calculate the filament cross section
            xs_slope = -2/(upper_line[0] + lower_line[0])
            xs_offset = img.height/2 - xs_slope*img.width/2

            #calculate the points of intersection of edges and filament cross section
            upper_intersect_x = (upper_line[1] - xs_offset)/(xs_slope - upper_line[0])
            upper_intersect_y = xs_slope*upper_intersect_x + xs_offset
            lower_intersect_x = (lower_line[1] - xs_offset)/(xs_slope - lower_line[0])
            lower_intersect_y = xs_slope*lower_intersect_x + xs_offset

            #calculate width of filament
            width = ((upper_intersect_y - lower_intersect_y)**2 + (upper_intersect_x - lower_intersect_x)**2)**0.5

            #Draw the fit lines and intersect markers 
            upper_layer = DrawingLayer((img.width, img.height))
            lower_layer = DrawingLayer((img.width, img.height))
            x_mid_layer = DrawingLayer((img.width, img.height))

            upper_start = upper_line[1]
            upper_stop = upper_line[0]*img.width + upper_line[1] 
            lower_start = lower_line[1]
            lower_stop = lower_line[0]*img.width + lower_line[1] 
            xSect_stop = xs_slope*img.width + xs_offset

            upper_layer.line((0,upper_start), (img.width, upper_stop), alpha=128, width=1, color=Color.RED)
            lower_layer.line((0,lower_start), (img.width, lower_stop), alpha=128, width=1, color=Color.RED)
            x_mid_layer.line((0, xs_offset), (img.width, xSect_stop), alpha=128, width=1, color=Color.RED)

            upper_marker = (int(upper_intersect_x),int(upper_intersect_y))
            lower_marker = (int(lower_intersect_x),int(lower_intersect_y))
            r = 10
            x_mid_layer.circle(upper_marker, r, alpha=128, filled=True, color=Color.RED)
            x_mid_layer.circle(lower_marker, r, alpha=128, filled=True, color=Color.RED)

            result.addDrawingLayer(upper_layer)
            result.addDrawingLayer(lower_layer)
            result.addDrawingLayer(x_mid_layer)
        except:
            print "Incomplete line data!, is the filament in place?"

        result.show()

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
#        output = combine(int(P + I + D), output_offset, forward)
#        output = clamp(output, out_min, out_max)

        if ARDUINO:
            if (t - last_out) > output_rate:
                sendMessage(connection, str(output))
                last_out = t

        print "width: " + str(width)
        print "set point: " + str(set_point)
        print "error: " + str(error)
        print "P: %d, I: %d, D: %d" % (int(P) , int(I) , int(D))
#        print "output: " + str(output)
        print "-------------"
    
       
    except (KeyboardInterrupt, SystemExit):
        if ARDUINO:
            sendMessage(connection, str(0))
            connection.close()
        raise
