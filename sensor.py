#!/usr/bin/python
'''
This program is basically the hello world in SimpleCV
all it does is grab an image from the camera and display it
'''
print __doc__

from SimpleCV import Image, Camera
cam = Camera()

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
    img.show()
