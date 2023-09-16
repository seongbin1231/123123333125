#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32

class SCANParser:
    def __init__(self):
        self.scan_sub=rospy.Subscriber("velodyne_points", PointCloud2, self.callback)

        self.dist_pub=rospy.Publisher("dist_forward",Float32, queue_size=10)

        self.pc_np=None

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list=[]
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            
            dist = np.sqrt(point[0]**2+point[1]**2+point[2]**2)

            angle=np.arctan2(point[1],point[0])

            if point[0]>0 and point[2]>-1.3 and dist <50:
                print(angle)
                point_list.append((point[0],point[1],point[2],point[3],dist,angle))

        point_np=np.array(point_list,np.float32)
        return point_np

        
    def calc_dist_forward(self):
        r_bool = (self.pc_np[:,5] > -30/180*np.pi) & (self.pc_np[:,5] < 30/180*np.pi)
        d_list = self.pc_np[r_bool,4]

        # d1=self.pc_np[r1_bool,4]
        # d2=self.pc_np[r2_bool,4]

        # d_list=np.concatenate([d1,d2])
        print(d_list,"@@@@@@@@@@@@@@@@@@@")
        if d_list.size==0:
            return 50.0
        return np.min(d_list)
    
    def callback(self,msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        
        # Print the shape of the pc_np array for debugging
        rospy.loginfo("Shape of pc_np: %s", str(self.pc_np.shape))
        
        d_min = self.calc_dist_forward()
        dist_msg = Float32()
        dist_msg.data = d_min
        print(self.pc_np)
        self.dist_pub.publish(dist_msg)


if __name__=="__main__":
    rospy.init_node("velodyne_parser",anonymous=True)

    scan_parser=SCANParser()

    rospy.spin()