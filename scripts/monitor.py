#!/usr/bin/env python

import rospy

def logic_monitor():
	rospy.init_node('monitor', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
    print("-------starting state monitor-------")
    
    logic_monitor()
