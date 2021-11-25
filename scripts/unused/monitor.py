#!/usr/bin/env python

import rospy

class logicMonitor:
    def __init__(self):
        print('init')

        self.monitorTimer=rospy.Timer(rospy.Duration(5),self.monitorCallback)

        
    def monitorCallback(self,event=None):
        print('timer')


if __name__ == '__main__':

    rospy.init_node('monitor',anonymous=True)
    print('starting monitor node')

    monitor=logicMonitor()
