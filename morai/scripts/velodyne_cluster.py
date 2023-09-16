#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
from sklearn.cluster import DBSCAN

class SCANCluster:
    def __init__(self):
        self.scan_sub=rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)

        self.cluster_pub=rospy.Publisher("clusters",Float64MultiArray, queue_size=10)
        
        self.cluster_pub2=rospy.Publisher("cluster_data",PointCloud2,queue_size=10)

        self.pc_np=None

        self.cluster_msg=Float64MultiArray()

        self.dbscan=DBSCAN(eps=0.5, min_samples=5)

    def callback(self,msg):
        # self.pc_np=self.pointcloud2_to_xyz(msg)

        # pc_xy=self.pc_np[:,:3]

        # db=self.dbscan.fit_predict(pc_xy)

        # n_cluster=np.max(db)+1

        # cluster_list=[]


        # xyz=self.pc_xy[db>=0,:]
        # print(xyz)

        self.pc_np = self.pointcloud2_to_xyz(msg)
        pc_xy = self.pc_np[:, :3]
        db = self.dbscan.fit_predict(pc_xy)
        xyz = pc_xy[db >= 0, :]
        self.xyz_array_to_pointcloud2(xyz)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        pointcloud2_msg = pc2.create_cloud_xyz32(header, xyz)

        self.cluster_pub2.publish(pointcloud2_msg)
        # for c in range(n_cluster):
        #     c_tmp=np.mean(pc_xy[db==c,:], axis=0)

        #     cluster_list+=c_tmp.tolist()

        # self.cluster_msg.data=cluster_list

        # self.cluster_pub.publish(self.cluster_msg)
    
    
    def pointcloud2_to_xyz(self, cloud_msg):
        point_list=[]
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            
            dist = np.sqrt(point[0]**2+point[1]**2+point[2]**2)

            angle=np.arctan2(point[1],point[0])
            
            if point[0]>0 and point[2]>-1.3 and dist <50:
                point_list.append(point)
                

        point_np=np.array(point_list,np.float32)
        return point_np

    def xyz_array_to_pointcloud2(self, points_array, frame_id="base_link"):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        pointcloud2_msg = pc2.create_cloud_xyz32(header, points_array)
        return pointcloud2_msg

    
    
    
    
    
    
    
    # def pointcloud2_to_xyz(self, cloud_msg):
    #     point_list=[]
    #     for point in pc2.read_points(cloud_msg, skip_nans=True):
            
    #         dist = np.sqrt(point[0]**2+point[1]**2+point[2]**2)

    #         angle=np.arctan2(point[1],point[0])

    #         if point[0]>0 and point[2]>-1.3 and dist <50:
    #             point_list.append((point[0],point[1],point[2],point[3],dist,angle))


    #     point_np=np.array(point_list,np.float32)
    #     return point_np
    
if __name__=="__main__":
    rospy.init_node("velodyne_cluster",anonymous=True)

    scan_parser=SCANCluster()

    rospy.spin()