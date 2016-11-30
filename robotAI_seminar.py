#!/usr/bin/python 
import rospy
import cv2 as cv

cap = cv.VideoCapture("/dev/video0")
face_cascade = cv.CascadeClassifier("")
