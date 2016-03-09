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
from SimpleCV import Image, Camera

def sendMessage(serial_connection, message):
    #Flush Serial buffer
    while serial_connection.inWaiting():
        flush_lines = serial_connection.readline()
    #Send status command
    serial_connection.write(message)
    # try:
    #     received_line = get_message(serial_connection)
    # except:
    #     raise
    # if received_line:
    #     print received_line

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


while True:
    img = cam.getImage()
    width = img.width
    height = img.height
   
    xSection = []
    dark_pixels = 0
    for y in range(height):
        current_pixel = img.getPixel(width/2,y)
        xSection.append(sum(current_pixel))
    for p in xSection:
        if p < 400:
            dark_pixels += 1
    print dark_pixels
    sendMessage(connection, str(dark_pixels))
    #print get_message(connection)
    img.show()
