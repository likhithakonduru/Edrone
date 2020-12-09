#!/usr/bin/env python

from vitarana_drone.msg import *
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import imutils
from pyzbar.pyzbar import decode
import pyzbar.pyzbar as decode
import pyzbar.pyzbar as pyzbar

class image_proc():

    # Initialise everything
    def __init__(self):
        # Subscribing to the camera topic
        rospy.init_node('qr_detect')
        # This will contain your image frame from camera
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.pub_coords = rospy.Publisher('/edrone/gps_coordinates', prop_speed, queue_size= 1)  #publishing the coordinates to position_controller using message type prop_speed
        self.values=[0.0 , 0.0 , 0.0]
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
    # Callback function of camera topic

    def image_callback(self, data):
        
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        decodedObjects = pyzbar.decode(self.img)
        # Print results
        for obj in decodedObjects:
        	self.values = obj.data.split(',')
        for x in range(3):
            self.values[x] = float(self.values[x]) 


    def coordinates(self):
        
        self.pub_coords.publish(self.values[0],self.values[1],self.values[2],0)     #publishing the coordinates
        if self.values[1] != 0.0 :
            rospy.loginfo(self.values)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
    	image_proc_obj.coordinates()
        r.sleep()
    	
        
      

       
