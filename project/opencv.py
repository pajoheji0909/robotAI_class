#!/usr/bin/python
import numpy as np
import cv2 as cv
import datetime as dt
import os
import time
import rospy
from std_msgs.msg import String

cwd = os.getcwd()
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
print ('openCV package loaded')
path = "~/Documents/robotAI/project"
valid_exts = [".jpg", ".gif", ".jpeg", ".png"]
cap = cv.VideoCapture('/dev/video0')
camera_logic = True


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


def run_camera(logic):
    if logic:
        ret, frame = cap.read()
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(frame_gray, 1.3, 5)
        for (x, y, w, h) in faces:
            cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv.imshow('frame', frame)
        cv.waitKey(10)

while (True):
    camera_logic = False
    run_camera(camera_logic)
    if camera_logic == False:
        cap.release()
        cv.destroyWindow()

    listener()
