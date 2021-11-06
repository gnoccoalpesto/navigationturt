#!/usr/bin/env python

import rospy
import os
import time
#usage: command=command_string; os.system(command)
import std_msgs.msg

class mapperClass:
    def __init__(self):
        self.completionTopic=rospy.get_param('~completed_exploration_topic','/exploration_completed')
        rospy.Subscriber(self.completionTopic,std_msgs.msg.Bool,self.mappingRequest,queue_size=1)
        self.mapName=rospy.get_param('~map_location','/catkin_ws/src/navigationturt/maps/map_')
        self.mappingCommand="rosrun map_server map_saver -f "

        print('==#== expection std_msgs.msg.Bool on {} ==#=='.format(self.completionTopic))
        print("==#== will save the map as "+self.mapName+"{request's date&time} ==#==")

    def mappingRequest(self,msg):
        if msg.data:
            # now = rospy.get_rostime()
            now=str()
            now=time.strftime("%Y%m%d_%H%M%S")
            os.system(self.mappingCommand+self.mapName+now)
            print('map saved as {}'.format(self.mapName+now))


if __name__=="__main__":

    print('==#== MAP SAVER NODE ==#==')
    rospy.init_node("mapper",anonymous=True)
    mapper=mapperClass()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('bye world')
        # pass
        
