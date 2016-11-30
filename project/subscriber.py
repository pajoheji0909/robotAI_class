#!/usr/bin python
import cv2
import sys
import rospy
import roslib
import std_msgs.msg
from geometry_msgs.msg import Point
import time
import numpy as np

class motor_controller:
	
	def __init__(self):
		rospy.init_node('motor_controller', anonymous=True)
		self.image_sub = rospy.Subscriber("/face", Point, self.callback)

	def callback(self, point):
		print point.x, point.y
                
def main(args):
  mc = motor_controller()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
