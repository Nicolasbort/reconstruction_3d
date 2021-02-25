#! /usr/bin/env python3

import time
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geographic_msgs.msg import *
from trajectory_msgs.msg import *
import math
import cv2
import numpy as np
from cv_bridge import CvBridge
from defines import *
from utils import *


bridge_ = CvBridge()

def image_pelican_callback(data):

    img = bridge_.imgmsg_to_cv2(data, "bgr8")

    cv2.imshow("Pelican", img)
    cv2.waitKey(30)



def image_hummingbird_callback(data):

    img = bridge_.imgmsg_to_cv2(data, "bgr8")

    cv2.imshow("Hummingbird", img)
    cv2.waitKey(30)




def get_images():

    while (True):
        rospy.loginfo("While")
        img_raw_pelican = rospy.wait_for_message("/pelican/camera_monocular/image_raw", Image, timeout=5)
        img_raw_humming = rospy.wait_for_message("/hummingbird/camera_monocular/image_raw", Image, timeout=5)

        img_pelican = bridge_.imgmsg_to_cv2(img_raw_pelican, "bgr8")   
        img_humming = bridge_.imgmsg_to_cv2(img_raw_humming, "bgr8")    

        cv2.imshow("pelican", img_pelican)
        cv2.imshow("humming", img_humming)

        cv2.waitKey(30)

if __name__ == "__main__": 
    rospy.init_node("mission_planner_node", anonymous=False)    
  
    get_images()

    rospy.spin()