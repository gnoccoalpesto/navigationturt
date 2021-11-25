#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from reference_listener import TFListener
import std_srvs.srv
import rosnode
import actionlib
# from actionlib.action_server import ActionServer
from move_base_msgs.msg import MoveBaseAction

class initialPositionSetter:

    def __init__(self,child,father,use_lookup,direct_transform):
        # topics to listen
        self.odometryListener=TFListener(child=child,father=father,\
                    use_lookup=use_lookup,direct_transform=direct_transform)
        # selects the initialization method
        self.PUSH_ON_INITIALPOSE=True
        # waits until move_base server is online
        self.MBClient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.MBClient.wait_for_server()
        # sets up the costmap cleaner routine
        self.costCleaner=rospy.ServiceProxy('move_base/clear_costmaps',\
                                             std_srvs.srv.Empty)
        #rosservice call /move_base/clear_costmaps "{}"
    
        if self.PUSH_ON_INITIALPOSE:
            pubCounter=0
            # initializes initialpose publisher
            self.initialPositionPusher=rospy.Publisher('initialpose',\
                geometry_msgs.msg.PoseWithCovarianceStamped, queue_size = 5)
            self.poseToPush=geometry_msgs.msg.PoseWithCovarianceStamped()
            rate=rospy.Rate(10)
            while not rospy.is_shutdown() and pubCounter<25:
                # pushes received odometry messase to the initialpose topic
                if self.odometryListener.messageRead:
                    self.poseToPush.header.frame_id='map'
                    # self.poseToPush.header.stamp = rospy.get_rostime()
                    self.poseToPush.pose.pose.position=self.odometryListener.transform.translation
                    self.poseToPush.pose.pose.orientation=self.odometryListener.transform.rotation
                    self.initialPositionPusher.publish(self.poseToPush)
                    # loop termination condition
                    pubCounter=pubCounter+1
                rate.sleep()
            #removes obstacles from costmap
            self.costCleaner()
            # kills itself
            rosnode.kill_nodes([rospy.get_name()])
        '''
        else:
            send empty request to /global_localization
        '''

if __name__=='__main__':

    rospy.init_node('initializer',anonymous=True)
    targetFrame=rospy.get_param('~target_frame','base_footprint')
    sourceFrame=rospy.get_param('~source_frame','odom')
    pos=initialPositionSetter(child=targetFrame,\
            father=sourceFrame,use_lookup=False,direct_transform=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('bye')