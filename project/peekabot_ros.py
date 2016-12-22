# !/usr/bin/env python
import rospy
import Image
import roslib
import cv2 as cv
import numpy as np
import RPI.GPIO as GPIO
from std_msgs.msg import Int32

robot_mode = -1;

h = 320
w = 320

start = time.time()
count = 1

def setting():
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


def image_display_face():
    cap = cv.VideoCapture(0) # put video device path
    face_cascade = cv.CascadeClassifier("/home/whentheycry/Documents/robotAI/project/haarcascade_frontalface_default.xml") # put cascade file path
    ret, frame = cap.read()
    cvt_frame = cv.cvtColor(frame, COLOR_BGR2GRAY)

    frame_rgb = cv.resize(frame, (320,320), fx = 1, fy = 1, interpolation = cv.INTER_AREA)
    timeLapsed = time.time() - start
    if timeLapsed > count:
        rgb = [0,0,0]
        r = g = b = 0
        for i in range (0,h,4):
            for j in range (0, w, 4):
                r = r + frame_rgb[i,j][2]
                g = g + frame_rgb[i,j][1]
                b = b + frame_rgb[i,j][0]
        rgb =  [r/(w*h/16),g/(w*h)/16,b/(w*h)/16]
        count = count + 1

        pwmR.start(rgb[0]*100/255)
        pwmG.start(rgb[1]*100/255)
        pwmB.start(rgb[2]*100/255)
        
    face_rec = face_cascade.detectMultiScale(cvt_frame, scaleFactor = 2.0, minSize = (30,30))
    image = Image.open('File.jpg') # put image file for find , found 
    image.show()
    

def image_display():
    image = Image.open('File.jpg') # put image file for searching , hiding 
    image.show()


def ros_Callback(mode_pub):
    rospy.loginfo(mode_pub.data)
    robot_mode = mode_pub.data
    

def ros_sub():
    rospy.init_node('peekabot_mode', anonymous = True)
    rospy.Subscriber('/mode_pub', Int32, ros_Callback)
    rospy.spin()
    

def main():
    ros_sub()
    temp_mode = robot_mode
    if temp_mode == 0 or temp_mode == 3:
        image_display()

    elif temp_mode == 2 or temp_mode == 4:
        image_display_face()

while(1):
    setting()
    main()
