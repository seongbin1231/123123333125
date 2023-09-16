#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError



class IMGParser:
    def __init__(self):
        self.image_sub=rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)
        self.click=rospy.Subscriber("/Click", Int32, self.check)
        self.signal=False
        self.count=1
    def callback(self,msg):
        try:
            np_arr=np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image=img_bgr
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)
    def check(self, msg):
        filename='./image/image'+str(self.count)+'.jpg'
        print(filename)
        cv2.imwrite(filename, self.image)
        self.count+=1
        

if __name__=="__main__":
    rospy.init_node("image_parser", anonymous=True)
    image_parser=IMGParser()
    rospy.spin()