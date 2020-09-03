
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
import time

import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, interp1d


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

    i = 0

    face_cascade = cv2.CascadeClassifier('/home/pongsakorn/tello_face_tracking/src/face_detection/src/haarcascade_frontalface_default.xml')

    ScaleFactor = 1.5
    minNeighbors = 3

    for frame in container.decode(video=0):
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

        res = cv2.resize(image, (320,240))
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)        
        faces = face_cascade.detectMultiScale(gray, ScaleFactor, minNeighbors)

        if (i > 900 and i <= 901):
            takeoff()

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
                    cmd_vel.angular.z = -0.4

                # middle_side
                elif center_x > width/3 and center_x <= 2*(width/3):
                    print("middle_side")
                    cmd_vel.angular.z = 0.0

                # right_side
                elif center_x > 2*(width/3) and center_x <= width:
                    print("right_side")
                    cmd_vel.angular.z = 0.4


                # move_forward
                if w > 0 and w <= width/6:
                    print("move_forward")
                    cmd_vel.linear.y = 0.3
                
                elif w> width/6 and w <= width/4:
                    print("no move")
                    cmd_vel.linear.y = 0

                # move backward 
                else:
                    print("move_backward")
                    cmd_vel.linear.y = -0.3
                                
                
                rospy.loginfo("center x: " + str(x+(w/2)))
                rospy.loginfo("center y: " + str(y+(h/2)))
                rospy.loginfo("frame_w: " + str(w))

        pub_vel.publish(cmd_vel)

        i = i+1
        print("iteration: " + str(i))

        cv2.imshow('Frame', res)
        cv2.waitKey(1)



def takeoff():
    print("taking off")
    pub_takeoff.publish(takeoff_msg) 
    rospy.sleep(1)
    
    cmd_vel.linear.z = 0.75
    pub_vel.publish(cmd_vel)

    rospy.sleep(3)

    cmd_vel.linear.z = 0
    pub_vel.publish(cmd_vel)   



class PID:

    def __init__(self, P=1.0, I=0.8, D=0.001, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):

        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time




if __name__ == '__main__':
    
    rospy.init_node('h264_listener')

    pub_takeoff = rospy.Publisher('/tello/takeoff',Empty,queue_size=1)

    pub_land = rospy.Publisher('/tello/land',Empty,queue_size=1)

    pub_vel = rospy.Publisher('/tello/cmd_vel',Twist,queue_size=1)

    takeoff_msg = Empty()
    cmd_vel = Twist()     

    try:

        cmd_vel.linear.z = 0.7
        pub_vel.publish(cmd_vel)
        
        rospy.sleep(3)

        cmd_vel.linear.z = 0
        pub_vel.publish(cmd_vel)

        main()

    except BaseException:
        traceback.print_exc()
    finally:
        pub_land.publish(Empty())

        stream.close()
        cv2.destroyAllWindows()








