#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import CollisionData
from morai_msgs.srv import MoraiEventCmdSrv
from morai_msgs.msg import EventInfo


class s_driver():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)

        cmd_pub=rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        event=rospy.ServiceProxy('/Service_MoraiEventCmd',MoraiEventCmdSrv)

        rate = rospy.Rate(30)

        self.collision=rospy.Subscriber('/CollisionData', CollisionData, self.callback)

        self.collisionflag=False
        cmd=CtrlCmd()
        cmd.longlCmdType=2
        cmd.velocity=20
        steering_cmd=[-1.0, 0.23]
        cmd_cnts=70
        s=EventInfo()

        while not rospy.is_shutdown():

            if self.collisionflag:
                s.option = 3
                s.ctrl_mode = 3 ##auto 3
                s.gear = 2
                event(s)
                for _ in range(60):
                    cmd_pub.publish(cmd)
                    rate.sleep()
                for _ in range(30):
                    cmd.velocity=0
                    cmd_pub.publish(cmd)
                    rate.sleep()
                s.option = 3
                s.ctrl_mode = 3 ##auto 3
                s.gear = 4
                event(s)
                cmd.velocity=20

                for i in range(2):
                    cmd.steering=steering_cmd[i]
                    for _ in range(cmd_cnts):
                        cmd_pub.publish(cmd)
                        rate.sleep()
                cmd.steering=0
                self.collisionflag=False
                ###
                #0=p, 1=m, 2=r, 3=n

                # print(s)
                

            # for i in range(2):
            #     cmd.steering=steering_cmd[i]
            #     for _ in range(cmd_cnts):
            cmd_pub.publish(cmd)
            rate.sleep()

    def callback(self,data):
        if len(data.collision_object)>0:
            self.collisionflag=True














if __name__=='__main__':
    try:
        s_d=s_driver()
    except rospy.ROSInternalException:
        pass