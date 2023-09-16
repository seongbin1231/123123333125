#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg

from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch


class IMGParser:
    def __init__(self):
        self.flag=0
        self.image_sub=rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)
        self.model=YOLO('yolov8n.pt')
        self.model=YOLO('morai_best.pt')
        self.flag=1
    def callback(self,msg):
        if self.flag==1:
            try:
                np_arr=np.fromstring(msg.data, np.uint8)
                
                img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                results = self.model.predict(img_rgb)
                for r in results:
                    annotator = Annotator(img_bgr)
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                        c = box.cls
                        annotator.box_label(b, self.model.names[int(c)])

            except CvBridgeError as e:
                print(e)
            frame=annotator.result()
            cv2.imshow("Image window", frame)
            cv2.waitKey(1)
        

if __name__=="__main__":
    device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
    print(device)
    print(torch.cuda.is_available())
    print(torch.version.cuda)
    rospy.init_node("image_parser", anonymous=True)
    image_parser=IMGParser()
    rospy.spin()