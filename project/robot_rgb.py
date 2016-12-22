# /usr/bin/env python
import cv2 
import time
import numpy as np
import RPI.GPIO as GPIO

height = 320
width  = 320

cap = cv2.VideoCapture(0)

start = time.time()
count = 1

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)
GPIO.output(16, GPIO.LOW)
GPIO.setup(20, GPIO.OUT)
GPIO.output(20, GPIO.LOW)
GPIO.setup(21, GPIO.OUT)
GPIO.output(21, GPIO.LOW)

pwmR = GPIO.PWM (20, 50)
pwmG = GPIO.PWM (16, 50)
pwmB = GPIO.PWM (21, 50)

while(1):
    ret, frame=cap.read()
    frame = cv2.resize(frame,(320,320),fx=1,fy=1,interpolation = cv2.INTER_AREA)

    cv2.imshow("frame", frame)

    timeLapsed = time.time() - start

    if timeLapsed > count:
        rgb = [0,0,0]
        r = g = b = 0
        for i in range (0, height, 4):
            for j in range (0, weight, 4):
                r = r + frame[i,j][2]
                g = g + frame[i,j][1]
                b = b + frame[i,j][0]
        rgb = [r/(width*height/16),g/(width*height)/16,b/(width*height)/16]
        print rgb
        count = count + 1

        pwmR.start(rgb[0]*100/255)
        pwmG.start(rgb[1]*100/255)
        pwmB.start(rgb[2]*100/255)

        if cv2.waitKey(1) == 20:
            break

cap.release()
cv2.destroyAllWindows()
