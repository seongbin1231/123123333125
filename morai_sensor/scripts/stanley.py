#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from gps_path import LL2UTMConverter
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
import numpy as np
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd
from math import sqrt, atan2, pi, sin, cos


class stanley():
    path_x=[]
    path_y=[]
    path_yaw=[]
    gps1_x, gps1_y=None, None
    gps2_x, gps2_y=None, None



    def __init__(self):
        rospy.init_node('stanley', anonymous=True)
        rospy.Subscriber("/gps1", GPSMessage, self.gps1)
        rospy.Subscriber("/gps2", GPSMessage, self.gps2)

        cmd_pub=rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.cmd=CtrlCmd()
        self.cmd.longlCmdType=2
        self.cmd.velocity=25
        self.cmd.steering=0.2

        path=open("./path/path.txt","r")
        lines=path.readlines()
        for line in lines:
            data=line.split()
            self.path_x.append(float(data[0]))
            self.path_y.append(float(data[1]))
        path.close()

        yaw=open("./path/yaw.txt","r")
        lines=yaw.readlines()
        for line in lines:
            self.path_yaw.append(float(line))
        yaw.close()


        rate = rospy.Rate(10)





        while not rospy.is_shutdown():
            rate.sleep()
            self.localization()
            self.heading()
            self.stanley()
            cmd_pub.publish(self.cmd)
            # print("gps1 x :",self.gps1_x,",gps1 y : ",self.gps1_y)
            # print("gps2 x :",self.gps2_x,",gps2 y : ",self.gps2_y)



    def localization(self):
        min=float("inf")
        # temp=0
        for i in range(len(self.path_x)):
            dx=self.gps1_x-self.path_x[i]
            dy=self.gps1_y-self.path_y[i]
            dist=sqrt(dx**2+dy**2)
            if dist<min:
                min=dist
                self.target_yaw=self.path_yaw[i]
                self.target_x=self.path_x[i]
                self.target_y=self.path_y[i]
                self.dx=dx
                self.dy=dy
        print(min)  #오차
    
    def heading(self):
        dx=self.gps1_x-self.gps2_x
        dy=self.gps1_y-self.gps2_y
        self.yaw=atan2(dy,dx)
        # print("heading : ",self.yaw)

    def stanley(self):
        theta_error=self.target_yaw-self.yaw
        if abs(theta_error)>pi:
            if theta_error>0:
                theta_error-=2*pi
            else:
                theta_error+=2*pi
        # print("theta_err",theta_error)


        e=-(self.dx*cos(self.yaw+pi/2)+self.dy*sin(self.yaw+pi/2))
        t_v=self.cmd.velocity/3.6

        t_v=max(t_v,5.0)
        dist_error=atan2(0.8*e,t_v)
        error=theta_error+dist_error
        # print(dist_error)
        np.clip(error,-1.0,1.0)
        self.cmd.steering=error
        # self.cmd.steering=theta_error



    def gps1(self,data):
        lat = data.latitude
        lon = data.longitude
        xy_zone=self.proj_UTM(lon, lat)
        self.gps1_x = xy_zone[0]
        self.gps1_y = xy_zone[1]
    def gps2(self,data):
        lat = data.latitude
        lon = data.longitude
        xy_zone=self.proj_UTM(lon, lat)
        self.gps2_x = xy_zone[0]
        self.gps2_y = xy_zone[1]




if __name__=='__main__':
    try:
        s_d=stanley()
    except rospy.ROSInternalException:
        pass