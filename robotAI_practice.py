import numpy as np
import cv2 as cv
import tensorflow as tf
import datetime as dt
import os
import time
# import rospy
# from std_msgs.msg import String
# from threading import Thread
# read current directory
cwd = os.getcwd()
# load face detector
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
print ('openCV package loaded')
print ('tensorflow package loaded')
path = "~/Documents/robotAI"
valid_exts = [".jpg", ".gif", ".jpeg", ".png"]

# time_1 = dt.datetime.now()
# time_2 = dt.datetime.now()
# time_elapsed = (time_2 - time_1)
# image read from directory
img_ex = cv.imread('Lenna.png')
# image show by new window
# cv.imshow('exp0', img_ex)
# wait for 0 ms
cv.waitKey(0)
# print image in matrix data
print img_ex
# set video device
cap = cv.VideoCapture('/dev/video0')
camera_logic = True
n = 0
time_ex0 = dt.datetime.now()
time_elapsed = 0


def timer(n):
    time_now = time.time()
    time_ex1 = dt.datetime.now()
    time_elapsed = (time_ex0 - time_ex1)
    # print time_now


def run_camera(camera_logic):
    # capture frame by frame
    if camera_logic:
        ret, frame = cap.read()
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # detect faces in the frame
        faces = face_cascade.detectMultiScale(frame_gray, 1.3, 5)
        # draw face rectangles
        for (x, y, w, h) in faces:
            cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv.imshow('frame', frame)
        cv.waitKey(1)

# background_thread = Thread(target=timer, args=n)
# background_thread.start()
# rospy.init_node("Helloworldnode")
# pub_test = rospy.Publisher('iterator', String, queue_size = 10)
# rate = rospy.Rate(100)
i = 0
while (True):
    timer(1)
    if time_elapsed == 10:
        camera_logic = False
        time_elapsed = 0
        cap.release()
        cv.destroyAllWindows()
       #  run_camera(camera_logic)
       #  pub_test.publish("Hello "+str(i))
    i = i + 1
    rate.sleep()
    
sess = tf.Session()
hello = tf.constant("Hello, it's TF")
sess.run(hello)
