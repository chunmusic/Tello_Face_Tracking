#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Range
import tellopy



class LoadPeople(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/tello/image_raw",Image,self.camera_callback)
        # self.hector_sub = rospy.Subscriber("/sonar_height", Range, self.sonar_callback)      
        self.bridge_object = CvBridge()        
        self.a = 0.0
        self.b = 0.0
        print("init")
        print(self.image_sub)
        print(self.camera_callback)


    # def sonar_callback(self, msg):
    #     self.a = msg.range 

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            print("try")
        except CvBridgeError as e:
            print(e)                      

        # hog = cv2.HOGDescriptor()
        # hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        
        #Size for the image 
        imX = 700
        imY = 500

        img_2 = cv2.resize(cv_image,(imX,imY))

        gray_2 = cv2.cvtColor(img_2, cv2.COLOR_RGB2GRAY)

        # boxes_2, weights_2 = hog.detectMultiScale(gray_2, winStride=(8,6) )
        # boxes_2 = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes_2])


        # for (xA, yA, xB, yB) in boxes_2:
            
        #     #Center in X 
        #     medX = xB - xA 
        #     xC = int(xA+(medX/2)) 

        #     #Center in Y
        #     medY = yB - yA 
        #     yC = int(yA+(medY/2)) 

        #     #Draw a circle in the center of the box 
        #     cv2.circle(img_2,(xC,yC), 1, (0,255,255), -1)

        #     # display the detected boxes in the colour picture
        #     cv2.rectangle(img_2, (xA, yA), (xB, yB),(255, 255, 0), 2)    
        # if self.a >1.96:
        #     if (xC < int(imX/2)-50 and self.b !=1):
        #         print (" The person is on the left")
        #         self.b =1
        #     if (xC > int(imX/2)-50 and xC < int(imX/2)+50 and self.b!=2):
        #         print (" The person is on the center")
        #         self.b = 2
        #     if (xC > int(imX/2)+50 and self.b !=3):
        #         print (" The person is on the right")
        #         self.b=3
        
        cv2.imshow('image',img_2)      

        rospy.loginfo("finished")   
         
          
        cv2.waitKey(1)



def main():
    load_people_object = LoadPeople()
    rospy.init_node('load_people_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
