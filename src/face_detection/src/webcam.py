#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
import av
import cv2
import numpy as np
import threading
import traceback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty



class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)



def main():
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    container = av.open(stream)
    rospy.loginfo('main: opened')

    width = 320
    height = 240

    face_cascade = cv2.CascadeClassifier('/home/pongsakorn/tello_face_tracking/src/face_detection/src/haarcascade_frontalface_default.xml')

    ScaleFactor = 1.2
    minNeighbors = 3

    for frame in container.decode(video=0):
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

        res = cv2.resize(image, (320,240))
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)        
        faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)


        if len(faces) == 0:
            print ("No face detected")
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
    

        else:
            for (x,y,w,h) in faces:
                
                cv2.rectangle(res,(x,y),(x+w,y+h),(255,0,0),2)  
                roi = res[y:y+h, x:x+w]

                center_x = (x+(w/2))
                center_y = (y+(h/2))
                
                # left_side
                if center_x > 0 and center_x <= width/3:
                    print("left_side")
                    cmd_vel.angular.z = -1

                # middle_side
                elif center_x > width/3 and center_x <= 2*(width/3):
                    print("middle_side")
                    cmd_vel.angular.z = 0.0

                # right_side
                elif center_x > 2*(width/3) and center_x <= width:
                    print("right_side")
                    cmd_vel.angular.z = 1
                                
                rospy.loginfo("center x: " + str(x+(w/2)))
                rospy.loginfo("center y: " + str(y+(h/2)))

        pub_vel.publish(cmd_vel)

        cv2.imshow('Frame', res)
        cv2.waitKey(1)



if __name__ == '__main__':
    
    pub_takeoff = rospy.Publisher('/tello/takeoff',Empty,queue_size=1)
    pub_takeoff.publish(Empty()) 

    pub_land = rospy.Publisher('/tello/land',Empty,queue_size=1)

    pub_vel = rospy.Publisher('/tello/cmd_vel',Twist,queue_size=1)

    cmd_vel = Twist()   

    try:
        main()

    except BaseException:
        traceback.print_exc()
    finally:
        pub_land.publish(Empty())

        stream.close()
        cv2.destroyAllWindows()
