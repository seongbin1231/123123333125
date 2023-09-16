#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from morai_msgs.msg import CtrlCmd

class s_driver():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)

        cmd_pub=rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        rate = rospy.Rate(30)

        cmd=CtrlCmd()
        cmd.longlCmdType=2
        cmd.velocity=20
        steering_cmd=[-1.0, 0.9]
        cmd_cnts=50

        while not rospy.is_shutdown():
            for i in range(2):
                cmd.steering=steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()




if __name__=='__main__':
    try:
        s_d=s_driver()
    except rospy.ROSInternalException:
        pass