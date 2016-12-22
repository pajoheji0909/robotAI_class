#!/usr/bin/env python
import sys
import rospy
import roslib
import cv2 as cv
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

mode_sel = 10 # implement the initial value


class face_detector:
	
	def __init__(self):
		rospy.init_node('face_detector', anonymous=True)
		self.face_pub = rospy.Publisher("/face", Point, queue_size=10)
		self.fd()

	def fd(self):
		cap=cv.VideoCapture("/dev/video0") # you have to choose right video device

		face_cascade=cv.CascadeClassifier("/home/whentheycry/Documents/robotAI/project/haarcascade_frontalface_default.xml") # you have to put right path for cascade file
		while(True):
			ret, frame=cap.read()
			frame_gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)

			faces=face_cascade.detectMultiScale(frame_gray,scaleFactor=1.3,minSize=(30,30))
			p = Point(-1, -1, -1)
			for (x,y,w,h) in faces:
				cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
				p.x = (x + w/2)
				p.y = (y + h/2)
                                p.z = h
			self.face_pub.publish(p)
			cv.imshow("frame",frame)

			cv.waitKey(1)

		cap.release()
		cv.destroyAllWindows()

                
class mode_subscriber:

    def __init__(self):
        rospy.init_node('mode_channel', anonymous=True)
        rospy.Subscriber("/mode_select",Int32, self.callback)
        
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id(), data.data)


def mode_publisher():
        global mode_sel
        i = 0
        mode_sel = i
        rospy.init_node('mode_pubPI', anonymous=True)
        pub = rospy.Publisher('mode', Int32, queue_size = 10)
        pub.publish(mode_sel)
        i = i + 1

        
def main(args):
  fd1  = face_detector()
  m_s = mode_subscriber()
  mode_publisher()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
