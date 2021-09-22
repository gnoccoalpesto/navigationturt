#!/usr/bin/env python

import rospy
import navigatioturt.msg
from geometry_msgs import Twist
import os
#import math
#import numpy as np
#from geometry_msgs.msg import Twist
# implement odometry msgs
# implement navigation msgs

'''
class navigator_node():
    def __init__(self):
        init_message = rospy.get_param('~message', 'hello')
        rate = float(rospy.get_param('~rate', '1.0'))
        top_vel = rospy.get_param('~topic_velocity', '/cmd_vel')
        top_vel_ty=float#???
        top_set = rospy.get_param('~topic_setpoint', '/Setpoint')
        rospy.loginfo('rate = %d', rate)
        pub_vel=rospy.Publisher(top_vel,top_vel_ty)
        msg_vel=top_vel_ty#??? more like data()

        while not rospy.is_shutdown():
            #calculate msg_vel

            #advertise msg_vel
            pub_vel.publish(msg_vel)

            rate.sleep()

'''
def callSetpoint():
    print('hi')

def navigate():
    print("navigating to the goal")
    ## NAVIGATION ALGORITHM ##
    # read setpoint topic


if __name__ == '__main__':


    print("------starting turtlebot navigator------")
    rospy.init_node('turtle_navigator',anonymous=True)

    top_vel_ty=Twist
    top_vel = rospy.get_param('~topic_velocity', '/cmd_vel')
    top_set = rospy.get_param('~topic_setpoint', '/Setpoint')
    top_odom = rospy.get_param('~topic_odometry', '/odom')

    # subscribe setpoint
    sub_set = rospy.Subscriber(top_set, navigatioturt.msg.Setpoint.msg, callSetpoint)
    # subscribe navigation data
#    sub_nav=rospy.Subscriber(top_nav,,callNavData)

    pub_vel=rospy.Publisher(top_vel,top_vel_ty)
#    msg_vel=top_vel_ty#??? more like data()

    while not rospy.is_shutdown():
        #calculate msg_vel

        #advertise msg_vel
        pub_vel.publish(msg_vel)

        rate.sleep()
    rate=rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            navigate()

            rate.sleep()

        print('navigation node closed')
        rospy.sleep(2)

    except KeyboardInterrupt:
        rospy.loginfo("turtle motion aborted!")

