# -*- coding: utf-8 -*-
# import the necessary packages

#<<<<<<<<Art By Ankit>>>>>>>>>>#

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
from pyzbar import pyzbar
import imutils
import argparse
import datetime
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)

lower = {'red':(170, 100, 0), 'green':(45, 100, 100), 'blue':(100,150, 0)}
upper = {'red':(180, 255, 255), 'green':(75, 255, 255), 'blue':(140, 255, 255)}

# define standard colors for circle around the object
colors = {'red':(0,0,255), 'green':(0,255,0), 'blue':(255,0,0)}

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
	help="path to output CSV file containing barcodes")
args = vars(ap.parse_args())

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))

# open the output CSV file for writing and initialize the set of
# barcodes found thus far
csv = open(args["output"], "w")
found = set()
# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
detR = detB = detG = detQR = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    frame = frame.array
    frame = imutils.resize(frame, width=400)
    frame1 = frame.copy()
    # find the barcodes in the frame and decode each of the barcodes
    barcodes = pyzbar.decode(frame1)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    detR = detB = detG = detQR = 0
    for key, value in upper.items():

            # construct a mask for the color from dictionary`1, then perform
            # a series of dilations and erosions to remove any small
            # blobs left in the mask
            kernel = np.ones((9,9),np.uint8)
            mask = cv2.inRange(hsv, lower[key], upper[key])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size. Correct this value for your obect's size
                if radius > 0.5:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                    cv2.putText(frame,key + " ball", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)
                    coldet = 1
                    if key == 'red':
                        detR = 1
                    if key == 'blue':
                        detB = 1
                    if key == 'green':
                        detG = 1

    for barcode in barcodes:
	# extract the bounding box location of the barcode and draw
	 # the bounding box surrounding the barcode on the image
        (x, y, w, h) = barcode.rect
        cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 0, 255), 2)
	# the barcode data is a bytes object so if we want to draw it
	# on our output image we need to convert it to a string first
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
	print(barcodeData)

	# draw the barcode data and barcode type on the image
        text = "{} ({})".format(barcodeData, barcodeType)
        cv2.putText(frame1, text, (x, y - 10),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

	# if the barcode text is currently not in our CSV file, write
	# the timestamp + barcode to disk and update the set
        if barcodeData not in found:
            csv.write("{},{}\n".format(datetime.datetime.now(),
		barcodeData))
            csv.flush()
            found.add(barcodeData)
        detQR = 1
        # ser.write(barcodeData)

    # show the frame
    #cv2.imshow("qrcode", frame1)
    #cv2.imshow("colors", frame)
    key = cv2.waitKey(1) & 0xFF
    if detR == 1:
        print(1)
        ser.write('1')
    elif detB == 1:
        print(2)
        ser.write('2')
    elif detG == 1:
        print(3)
        ser.write('3')
    elif detQR == 1:
        print(barcodeData)
        ser.write('4')
    else:
        print(0)
        #3ser.write('0')
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
cv2.destroyAllWindows()
