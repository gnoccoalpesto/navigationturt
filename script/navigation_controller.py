#!/usr/bin/env python

import rospy
#import math
#import numpy as np
#from geometry_msgs.msg import Twist
# implement odometry msgs
# implement navigation msgs

exploration_completed=False
exploration_counter=1
EXPLORATION_GOAL_ITERACTIONS=1000

def explore():
    global exploration_completed
    global exploration_counter
    print("exploring the environment, iteration: "+str(exploration_counter))
    ## EXPLORATION ALGORITHM ##
    
    ## EXPLORATION END ##
    if exploration_counter>=EXPLORATION_GOAL_ITERACTIONS:
        exploration_completed=True
        print("EXPLORATION COMPLETED")
    else:
        exploration_counter=exploration_counter+1
    
def navigate():
    print("navigating to the goal")
    ## NAVIGATION ALGORITHM ##
    # read setpoint topic
     
    
if __name__ == '__main__':
    print("------starting turtlebot control------")
    rospy.init_node('turtle_motion',anonymous=True)
    
    
    # subscribe setpoint
    # subscribe navigation data
    rate=rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            exploration_mode=rospy.get_param("/exploration_mode",True)
            if exploration_mode:
                explore()
                if exploration_completed:
                    break
            else:
                navigate()
            
            rate.sleep()
            
        print('navigation node closed')
        rospy.sleep(2)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("turtle motion aborted!")
        
