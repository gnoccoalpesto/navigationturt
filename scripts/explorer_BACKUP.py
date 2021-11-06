#!/usr/bin/env python

import rospy
import actionlib
import math
import random
import time
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal#,MoveBaseActionFeedback
from geometry_msgs.msg import Twist
# from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#####################################################
# prevents undesired output
# from contextlib import contextmanager
# import sys, os
# @contextmanager
# def suppress_stdout():
#     with open(os.devnull, "w") as devnull:
#         old_stdout = sys.stdout
#         sys.stdout = devnull
#         try:  
#             yield
#         finally:
#             sys.stdout = old_stdout
###################################################
class coord2D:
    def __init__(self,x=0,y=0,teta=0):
        self.x=x
        self.y=y
        self.teta=teta

class explorationNode:
    def __init__(self):
        # topics to listen
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')
        self.mapTopic= rospy.get_param('~map_topic','/map')
        self.completionTopic=rospy.get_param('~completed_exploration_topic','/exploration_completed')
        self.velocityTopic= rospy.get_param('~velocity_topic','/cmd_vel')

        self.explorationCompleted=False
        self.robotIsStuck=False
        self.explorationIteration=0
        self.collisionCounter=0
        self.proximityCounter=0
        self.frontiers=[]
        self.map=OccupancyGrid()
        # self.referenceWpoint=coord2D()# reference WORLD point
        # self.referenceMpoint=coord2D()# reference MAP point

        self.UNKNOWN_THRESHOLD=1# min number of unknown neighbour to look for
        self.FREESPACE_THRESHOLD=1# min number of surrounding free space tiles to look for
        self.OBSTACLE_DISTANCE=0.4# max distance [meters] to consider an obstacle
        self.MAX_LINEAR_VELOCITY=0.22#[meters/seconds] of the robot
        self.MAX_ROTATION_VELOCITY=2.84#[rad/s] of the robot {==162.72 deg/s}
        self.COLLISION_DISTANCE=0.16# motion needed [meters] to consider robot not stuck
        self.COLLISION_ANGLE=12# rotation angle [degrees] under which robot is considered stuck
        self.MAX_COLLISION_COUNT=18# max allowed iterations count to determine a collision
        self.GOAL_DISTANCE=0.6# max distance [meters] to which robot is "close to goal"
        self.MAX_PROXIMITY_COUNT=17# max allowed iterations count to determine proximity to goal

        # initialize map
        self.initializeMap()
        # initialize move_base
        self.MBClient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.MBClient.wait_for_server()
        self.MBGoal=MoveBaseGoal()
        # initiate odometry listening
        print('__ starting environment exploration __\n')
        self.poseListener=rospy.Subscriber(self.odometryTopic,Odometry,self.poseCallback)
        NODE_START_DELAY=rospy.get_param('node_start_delay',default=3.0)
        self.explorationDuration=time.time()-NODE_START_DELAY
        # initiate exploration call; single shot to avoid multiple instances
        self.explorationTimer=rospy.Timer(rospy.Duration(NODE_START_DELAY),\
            self.explorationTimerCallback,oneshot=True)
        # velocity broadcaster for full rotation
        self.velocityChatter=rospy.Publisher(self.velocityTopic,Twist,queue_size=1)

    # MAP #######################################################

    def initializeMap(self):
    # request map informations
        map=rospy.wait_for_message(self.mapTopic,OccupancyGrid,timeout=rospy.Duration(20))
        self.mapHeight=map.info.height
        self.mapWidth=map.info.width
        self.mapOrigin=coord2D(map.info.origin.position.x,map.info.origin.position.y,\
            euler_from_quaternion((map.info.origin.orientation.x,map.info.origin.orientation.y,\
                            map.info.origin.orientation.z,map.info.origin.orientation.w))[2])
        self.mapResolution=round(map.info.resolution,4)
        print('##############\nMAP DATA:\nheight: {}\twidth: {}\norigin: {}, {}\norientation: {}\nresolution: {}\n#############'\
            .format(self.mapHeight,self.mapWidth,self.mapOrigin.x,self.mapOrigin.y,self.mapOrigin.teta,self.mapResolution))
        self.mapTiles=self.mapHeight*self.mapWidth
        
    def updateMap(self):
    # update map data
        del self.map# self.map.data.clear
        map=rospy.wait_for_message(self.mapTopic,OccupancyGrid)
        self.map=map.data

    # MOTION ############################################################

    def requestNewGoal(self,point,rpy=(0,0,0),barrell_roll=False):
    # construct move_base goal request
        self.MBGoal.target_pose.header.frame_id="map"
        self.MBGoal.target_pose.header.stamp=rospy.Time.now()
        self.MBGoal.target_pose.pose.position.x=point.x
        self.MBGoal.target_pose.pose.position.y=point.y
        self.MBGoal.target_pose.pose.position.z=0
        (qx,qy,qz,qw)=quaternion_from_euler(*rpy)
        self.MBGoal.target_pose.pose.orientation.x=qx
        self.MBGoal.target_pose.pose.orientation.y=qy
        self.MBGoal.target_pose.pose.orientation.z=qz
        self.MBGoal.target_pose.pose.orientation.w=qw
        self.MBClient.send_goal(self.MBGoal)
        # if not looking around
        if not barrell_roll:
            referenceDistance=self.pointDistance(self.referenceWpoint,self.nextWGoal)
            timeToReachGoal=56+int(1.7*referenceDistance/self.MAX_LINEAR_VELOCITY)
            # call motion monitor
            self.monitor=rospy.Timer(rospy.Duration(0.8),self.motionMonitor)
            if not self.robotIsStuck:
                print('goal sent, waiting {} seconds for result'.format(timeToReachGoal))
        else: timeToReachGoal= 10
        requestResult=self.MBClient.wait_for_result(rospy.Duration(timeToReachGoal))
        if not barrell_roll: self.monitor.shutdown()
        self.resetMonitor()
        return requestResult

    def waitCurrentPose(self):
    # wait until /odom is published, then gathers pose data
        odomPose=rospy.wait_for_message(self.odometryTopic,Odometry,timeout=rospy.Duration(20))
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        teta=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        return coord2D(x,y,teta)

    def updateReferencePose(self):
    # updates reference positions
        # reference position in WORLD frame
        self.previousWpoint=self.referenceWpoint=self.waitCurrentPose()
        # reference position in MAP frame
        self.referenceMpoint=self.convertToMapPosition(self.referenceWpoint)

    def poseCallback(self,odomPose):
    # updates current position
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        teta=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        # current WORLD point from odometry
        self.currentWpoint=coord2D(x,y,teta)


    # EXPLORATION ##########################################################
    
    def explorationTimerCallback(self,event=None):
    # periodic exploration algorithm
        # reference position in WORLD frame and MAP frame
        self.updateReferencePose()
        # look around performing a full rotation in place
        # NEED TO CHECK IS STUCK WHILE ROTATING
        self.doABarrellRoll()
        # compute frontier points
        if not self.localizeFrontiers():
            # broadcast exploration completion
            self.explorationCompleted=True
            completionMessage=Bool()
            completionMessage.data=True
            # SUBSTITUTE WITH SERVICE TO AVOID HAVING ANOTHER TERMINAL FOR MAPPER FEEDBACK
            self.completionChatter=rospy.Publisher(self.completionTopic,Bool,queue_size=1)#,latch=True)
            self.completionChatter.publish(completionMessage)
            # self.explorationTimer.shutdown()
            self.explorationDuration=time.time()-self.explorationDuration
            print('no frontier point found: EXPLORATION COMPLETED in {} seconds\
                ({} iterations)'.format(self.explorationDuration,self.explorationIteration))
            input('busy wait')
        else:        
            self.explorationIteration+=1
            print('\n----\nITERATION: {}\nreference point {}, {} ({}, {})'.format(self.explorationIteration,\
                self.referenceWpoint.x,self.referenceWpoint.y,self.referenceMpoint.x,self.referenceMpoint.y))
            # compute the next frontier to explore
            self.computeNextGoal()
            # start goal distance computation
            # self.goalMeter=rospy.Timer(rospy.Duration(0.5),self.goalDistance())
            print('next point to explore: {}, {} ({}, {})\n'.format(\
                    self.nextWGoal.x,self.nextWGoal.y,self.nextMGoal.x,self.nextMGoal.y))
            # send goal to move_base       
            if self.requestNewGoal(self.nextWGoal):
                if not self.robotIsStuck:print('goal successffully reached!')
                # self.MBClient.get_state()
            else: print('Move_Base cannot reach the goal!')
            # self.goalMeter.shutdown()
            # start a new instance of the timer to perform this callback
            self.explorationTimer=rospy.Timer(rospy.Duration(0.1),self.explorationTimerCallback,oneshot=True)
    
    def localizeFrontiers(self):
    # frontier points identification
        self.updateMap()
        self.frontiers=[]
        self.chartedTiles=0
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                # self.isCharted(x,y)
                self.chartedTiles+=self.isCharted(x,y)
                if self.isEmptySpace(x,y) and self.hasFreeSpaceAround(x,y)\
                and self.hasUnknownNeighbour(x,y):
                    self.frontiers.append((x,y))
        if self.frontiers==[]: return False
        else: return True

    def computeNextGoal(self):
    # compute the next frontier to explore
        # https://jakevdp.github.io/PythonDataScienceHandbook/02.08-sorting.html
        self.frontiers.sort(key=\
            lambda x:(x[0]-self.referenceMpoint.x)**2+(x[1]-self.referenceMpoint.y)**2)
        filteredFrontiers=filter(lambda x:(x[0]-self.referenceMpoint.x)**2+\
            (x[1]-self.referenceMpoint.y)**2>int(1/self.mapResolution),self.frontiers)
        if not filteredFrontiers:
            filteredFrontiers=filter(lambda x:(x[0]-self.referenceMpoint.x)**2+\
                (x[1]-self.referenceMpoint.y)**2>int(0.25/self.mapResolution),self.frontiers)
        self.frontiers=filteredFrontiers
        # if self.explorationIteration%5==0: chosenIndex=0
        # else: chosenIndex=int(len(self.frontiers)/3)
        chosenIndex=int(len(self.frontiers)/(3+self.selectionCoefficient))
        self.nextMGoal=coord2D(self.frontiers[chosenIndex][0],self.frontiers[chosenIndex][1],self.currentWpoint.teta)            
        self.nextWGoal=self.convertToWorldPosition(self.nextMGoal)
        
    def selectionCoefficient(self):
        chartedPercentage=self.chartedTiles/self.mapTiles
        return 2 if chartedPercentage>0.4 else 2*chartedPercentage
        
        
    # SECONDARY BEHAVIOUR ###############################################################
    
    def doABarrellRoll(self):
    # performs a complete rotation in stationary position
        print('looking around\n')
        rotationMsg=Twist()
        rotationSpeed=self.MAX_ROTATION_VELOCITY/2
        rotationMsg.angular.z=rotationSpeed
        self.velocityChatter.publish(rotationMsg)
        rospy.sleep(2*math.pi/rotationSpeed)
        rotationMsg.angular.z=0
        self.velocityChatter.publish(rotationMsg)

    def randomMotion(self):
    # performs a random rotation followed by a random linear motion
        print('performing a motion in random direction\n')
        velocityMessage=Twist()
        # random rotation
        speed=self.MAX_ROTATION_VELOCITY/2
        velocityMessage.angular.z=speed
        self.velocityChatter.publish(velocityMessage)
        rospy.sleep(2*math.pi*round(random.uniform(0,1),4)/speed)
        # random linear motion
        velocityMessage.angular.z=0
        velocityMessage.linear.x=self.MAX_LINEAR_VELOCITY*0.8
        self.velocityChatter.publish(velocityMessage)
        rospy.sleep(round(random.uniform(0,3),4))
        velocityMessage.linear.x=0
        self.velocityChatter.publish(velocityMessage)

    def motionMonitor(self,event=None):
    # monitor correct motion
        self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
        # checks if robot is taking too long to reach the goal from a close position
        if self.proximityCheck(): self.MBClient.cancel_goal()
        # checks if robot is blocked in a given position for too long
        if self.collisionCheck():# print('collision')
            if not self.robotIsStuck:
                self.monitor.shutdown()
                self.resetMonitor()
                # self.MBClient.cancel_goal()
                # self.randomMotion()
                # self.requestNewGoal(self.referenceWpoint)
                # print('going back to starting point')
                self.robotIsStuck=True

    def resetMonitor(self):
    # resets motion monitor's counters and state
        self.collisionCounter=0
        self.proximityCounter=0
        self.robotIsStuck=False
        self.closeToTarget=False

    def collisionCheck(self):
    # checks if the robot stays close to the same point for too long, hence counts
        if self.pointDistance(self.currentWpoint,self.previousWpoint)<self.COLLISION_DISTANCE\
        or abs(self.currentWpoint.teta-self.previousWpoint.teta)<self.COLLISION_ANGLE:
            self.collisionCounter+=1
        else: 
            # if not, start over
            self.robotIsStuck=False
            self.collisionCounter=0
            self.previousWpoint=self.currentWpoint
        # if robot close to target, wait much longer
        if self.collisionCounter>self.MAX_COLLISION_COUNT and not self.closeToTarget\
        or self.collisionCounter>self.MAX_COLLISION_COUNT*2 and self.closeToTarget:
            self.collisionCounter=0
            print('robot is stuck near point {}, {}, with orientation {}'\
                .format(self.previousWpoint.x,self.previousWpoint.y,self.previousWpoint.teta))
            return True
        else: return False

    def proximityCheck(self):
    # checks if robot is close, yet not at, the target
        if self.distanceFromGoal<self.GOAL_DISTANCE:
            self.closeToTarget=True
            # if it's close enough, counts
            if self.distanceFromGoal<self.GOAL_DISTANCE*0.42:
                self.proximityCounter+=1
            # otherwise starts over
            else: self.proximityCounter=0
        else: self.closeToTarget=False
        if self.closeToTarget and self.proximityCounter>self.MAX_PROXIMITY_COUNT:
            print('Robot is close enough to the target\n')
            return True
        else: return False


    # FRONTIER POINTS DEFINITION ################################################

    def isEmptySpace(self,point_x,point_y):
    # checks if a poit is empty space
        return self.map[point_y*self.mapWidth+point_x]==0

    def hasUnknownNeighbour(self,point_x,point_y,connected8=False):
    # checks if enough neighboors are still uncharted space
        neighbourCounter=0
        if not connected8:
        # 4-connected neighbourhood (N,S,E,W)
            neighbours=[(point_x+1, point_y), (point_x, point_y+1),\
                        (point_x, point_y-1), (point_x-1, point_y)]
        else:
        # 8-connected neighbourhood (also diagonals)
            neighbours=[(point_x+1, point_y), (point_x, point_y+1),\
                    (point_x, point_y-1), (point_x-1, point_y),\
                    (point_x+1, point_y-1), (point_x+1, point_y+1),\
                    (point_x-1, point_y-1), (point_x-1, point_y+1)]
        for neigh in neighbours:
            if self.map[neigh[1]*self.mapWidth+neigh[0]]==-1:
                neighbourCounter+=1

        if (not connected8 and neighbourCounter>=self.UNKNOWN_THRESHOLD)\
        or (connected8 and neighbourCounter>=2*self.UNKNOWN_THRESHOLD): 
            return True
        else: return False

    def hasFreeSpaceAround(self,point_x,point_y):
    # checks if a possible frontier has obstacles too close
        freespaceCounter=0
        obstacleMapDistance=int(self.OBSTACLE_DISTANCE/self.mapResolution)
        # in a specific square around the candidate point
        for query in range(-obstacleMapDistance,obstacleMapDistance):
            query_x=point_x+query
            query_y=point_y+query
            if query_x>0 and query_x<self.mapWidth and query_y>0 and query_y<self.mapHeight:
                if not self.map[query_y*self.mapWidth+query_x]>0: freespaceCounter+=1
        if freespaceCounter>=self.FREESPACE_THRESHOLD: return True
        else: return False


    # UTILITIES ###################################################################

    def isCharted(self,point_x,point_y):
    # checks if a tile has been charted alreasy
        return 0 if self.map[point_y*self.mapWidth+point_x]==-1 else 1
        # if not self.map[point_y*self.mapWidth+point_x]==-1:
            # self.chartedTiles+=1

    def convertToMapPosition(self,position):
    # converts coord2D WORLD -> MAP
        return coord2D(int((position.x-self.mapOrigin.x)/self.mapResolution),\
                        int((position.y-self.mapOrigin.y)/self.mapResolution),position.teta)    
                        
    def convertToWorldPosition(self,position):
    # converts coord2D MAP -> WORLD
        return coord2D(position.x*self.mapResolution+self.mapOrigin.x,\
                        position.y*self.mapResolution+self.mapOrigin.y,position.teta)

    def goalDistance(self,event=None):
        self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
            
    def pointDistance(self,pointA,pointB):
        return math.sqrt((pointA.x-pointB.x)**2+(pointA.y-pointB.y)**2)

    # def distances(self):
    #     newlist=[]
    #     for index in range(len(self.frontiers)):
    #         newlist.append((math.sqrt((self.frontiers[index][0]-self.referenceMpoint.x)**2+\
    #             (self.frontiers[index][1]-self.referenceMpoint.y)**2))*self.mapResolution)

    #     print(newlist)

    def showFrontiers(self,message=""):
    # prints frontiers on screen
        print('###############################')
        if not message=="":print('--- {} ---'.format(message))
        for c in range(len(self.frontiers)):
            pointMx,pointMy=self.frontiers[c][0],self.frontiers[c][1]
            pointW=self.convertToWorldPosition(coord2D(pointMx,pointMy))
            print('{} frontier: {} ({}) , {} ({}) \n'.\
                format(c,pointW.x,pointMx,pointW.y,pointMy))
        print('###############################')
        rospy.sleep(30)# time to read!


###################################################################################

if __name__ == '__main__':
    print('==#== ENVIRONMENT EXPLORATION NODE ==#==\n')
    rospy.init_node('explorer',anonymous=True)
    explorer=explorationNode()
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("turtle exploration aborted!")