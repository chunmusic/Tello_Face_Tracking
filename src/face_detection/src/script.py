#!/usr/bin/env python

import socket
import time

import cv2


tello_ip = '192.168.10.1'
tello_port = 8889
tello_address = (tello_ip, tello_port)

host = '192.168.10.10'
port = 11111
mypc_address = (host, port)


socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket.bind(mypc_address)

socket.sendto('command'.encode('utf-8'), tello_address)

socket.sendto('streamon'.encode('utf-8'), tello_address)
print("start streaming")

capture = cv2.VideoCapture('udp://192.168.10.2:11111')
while True:
    ret, frame = capture.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
capture.release()
cv2.destroyAllWindows()

socket.sendto('streamoff'.encode('utf-8'), tello_address)