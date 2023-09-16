#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import tf
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32

from morai_msgs.msg import GPSMessage, EgoVehicleStatus

#WGS위도 경도 좌표를 UTM좌표로 변환

class LL2UTMConverter():
    def __init__(self, zone=52):

        self.gps_sub = rospy.Subscriber("/gps1", GPSMessage, self.navsat_callback)
        self.x, self.y = None, None
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False) #preserve false == Meter
        self.f=open("path.txt","w")
        self.write_call=rospy.Subscriber("/writecall", Int32, self.stop)
        self.flag=True

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        # rospy.loginfo("latitude: %f, longitude: %f",self.lat,self.lon)

        self.e_i = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.convertLL2UTM()

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 0.),
                        tf.transformations.quaternion_from_euler(0, 0, 0,),
			            rospy.Time.now(),
			            "map",
			            "base_link")
	
        utm_msg = Float32MultiArray()

        utm_msg.data = [self.x, self.y]
        #print(utm_msg)
    def convertLL2UTM(self):
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        if self.flag:
            # rospy.loginfo("utm x: %f, utm y: %f",self.x,self.y)
            data = str(self.x) + "\t" + str(self.y)+"\n"
            self.f.write(data)


    def stop(self):
        self.flag=False
        self.f.close()


if __name__ == '__main__':

	rospy.init_node('gps_parser', anonymous=True)

	gps_parser = LL2UTMConverter()

	rospy.spin()
        
