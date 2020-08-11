#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import cv2
import numpy as np


class LoadVideo(object):

    def __init__(self):
        self.ctrl_c = False 
       
        self.bridge_object = CvBridge()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def video_detection (self,):
        # cap = cv2.VideoCapture("/home/user/opencv_for_robotics_images/Unit_3/Course_images/chris.mp4")
        cap = cv2.VideoCapture(0)
        face_cascade = cv2.CascadeClassifier('/home/pongsakorn/drone_face_tracking/src/face_detection/src/haarcascade_frontalface_default.xml')

        ScaleFactor = 1.2
        
        minNeighbors = 3

        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = 0.2


        width  = cap.get(3) # float
        height = cap.get(4) # float

        while not self.ctrl_c:

            ret, frame = cap.read()         
            img = frame

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)        

            faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)
            
            for (x,y,w,h) in faces:
                
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)  
                roi = img[y:y+h, x:x+w]

                center_x = (x+(w/2))
                center_y = (y+(h/2))
                
                # rospy.loginfo("center x: " + str((x+w)/2))
                # rospy.loginfo("center y: " + str((y+h)/2))

                # right_side
                if center_x > 0 and center_x <= width/3:
                    print("left_side")

                elif center_x > width/3 and center_x <= 2*(width/3):
                    print("middle_side")

                elif center_x > 2*(width/3) and center_x <= width:
                    print("right_side")

                rospy.loginfo("center x: " + str(x+(w/2)))
                rospy.loginfo("center y: " + str(y+(h/2)))

                
                pub.publish(cmd_vel)


            cv2.imshow('Face',img)
               
            cv2.waitKey(1)
        cap.release()
 

if __name__ == '__main__':
    
    rospy.init_node('load_video_node')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=20)
    load_video_object = LoadVideo()

    cmd_vel = Twist()   

    try:
        load_video_object.video_detection()
        rospy.oncespin()
    except rospy.ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()