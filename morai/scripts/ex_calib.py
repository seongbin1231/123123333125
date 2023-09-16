#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
import rospkg
import os
import json
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridgeError

class calib:
    def __init__(self):

        self.scan_sub=rospy.Subscriber("/velodyne_points", PointCloud2, self.vel_callback)
        self.image_sub=rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.img_callback)


        self.RT=transformMTX_lidar2cam()
        self.proj_mtx=project2img_mtx()

        self.pc_np=None

    
    def transform_lidar2cam(self,xyz_p):
        xyz_c=np.matmul(np.concatenate([xyz_p,np.ones((xyz_p.shape[0],1))],axis=1),self.RT.T)
        return xyz_c
    
    def project_pts2img(self, xyz_c, crop=True):
        xyz_c=xyz_c.T
        xc, yc, zc=xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]),xyz_c[2,:].reshape([1,-1])

        xn,yn=xc/(zc+0.0001), yc/(zc+0.0001)

        xyi=np.matmul(self.proj_mtx,np.concatenate([xn,yn,np.ones_like(xn)],axis=0))
        xyi=xyi[0:2,:].T

        if crop:
            xyi=self.crop_pts(xyi)
        else:
            pass
        return xyi
    
    def crop_pts(self,xyi):
        xyi=xyi[np.logical_and(xyi[:,0],xyi[:,0]<640),:]
        xyi=xyi[np.logical_and(xyi[:,1],xyi[:,1]<480),:]

        return xyi


    def img_callback(self,msg):
        try:
            np_arr=np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_bgr=draw_pts_img(img_bgr,self.xyi[:,0],self.xyi[:,1])
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1)

    def vel_callback(self,msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        xyz_c=self.transform_lidar2cam(self.pc_np)
        self.xyi=self.project_pts2img(xyz_c)
    

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list=[]
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            
            dist = np.sqrt(point[0]**2+point[1]**2+point[2]**2)
            
            if point[0]>0 and point[2]>-1.3 and dist <50:
                point_list.append((point[0],point[1],point[2]))
                

        point_np=np.array(point_list,np.float32)
        
        return point_np





def transformMTX_lidar2cam():
    x_rel=2.74-1.02
    y_rel=0
    z_rel=0.72-1.22

    R_T=np.matmul(translationMtx(x_rel,y_rel,z_rel), rotationMtx(np.deg2rad(-90.),0.,0.))
    R_T=np.matmul(R_T,rotationMtx(0.,0.,np.deg2rad(-90.)))

    R_T=np.linalg.inv(R_T)

    return R_T

def project2img_mtx():
    fc_x=640/(2*np.tan(np.deg2rad(45)))
    fc_y=640/(2*np.tan(np.deg2rad(45)))

    cx=640/2
    cy=480/2

    return np.array([[fc_x,0,cx],[0,fc_y,cy]])

def draw_pts_img(img, xi, yi):
    point_np = img

    for ctr in zip(xi, yi):
        # Convert the coordinates to integers
        ctr_int = (int(ctr[0]), int(ctr[1]))
        point_np = cv2.circle(point_np, ctr_int, 2, (255, 0, 0), -1)

    return point_np








def translationMtx(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def rotationMtx(yaw, pitch, roll):
    R_x = np.array([[1, 0, 0, 0],
                    [0, math.cos(roll), -math.sin(roll), 0],
                    [0, math.sin(roll), math.cos(roll), 0],
                    [0, 0, 0, 1]])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch), 0],
                    [0, 1, 0, 0],
                    [-math.sin(pitch), 0, math.cos(pitch), 0],
                    [0, 0, 0, 1]])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0, 0],
                    [math.sin(yaw), math.cos(yaw), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    return np.matmul(R_x, np.matmul(R_y, R_z))


if __name__=="__main__":
    rospy.init_node("calib", anonymous=True)
    image_parser=calib()
    rospy.spin()