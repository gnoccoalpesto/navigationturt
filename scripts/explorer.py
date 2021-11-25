#!/usr/bin/env python

import rospy
import actionlib
import math
import random
import time
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction#, MoveBaseGoal#,MoveBaseActionFeedback
from geometry_msgs.msg import Twist
# from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion#, quaternion_from_euler
from action_move_base import navigationNode, coord2D
import numpy as np
# from cv2 import connectedComponents
# pip2 install opencv-python==4.2.0.32 not recognised here
# pip install opencv-python

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

class explorationNode:
    def __init__(self):
        # topics to listen
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')
        self.mapTopic= rospy.get_param('~map_topic','/map')
        self.completionTopic=rospy.get_param('~completed_exploration_topic','/exploration_completed')
        self.velocityTopic= rospy.get_param('~velocity_topic','/cmd_vel')

        self.explorationCompleted=False
        self.explorationIteration=0
        self.frontiers=[]
        self.map=OccupancyGrid()
        self.UNKNOWN_THRESHOLD=1# min number of unknown neighbour to look for
        self.FREESPACE_THRESHOLD=1# min number of surrounding free space tiles to look for
        self.OBSTACLE_DISTANCE=0.4# max distance [meters] to consider an obstacle
        
        # initialize map
        self.initializeMap()
        # initialize move base client node
        self.navigator=navigationNode()
        # velocity broadcaster for full rotation
        self.velocityChatter=rospy.Publisher(self.velocityTopic,Twist,queue_size=1)
        # initiate odometry listening
        self.poseListener=rospy.Subscriber(self.odometryTopic,Odometry,self.poseCallback)
        NODE_START_DELAY=rospy.get_param('node_start_delay',default=3.0)
        self.explorationDuration=time.time()-NODE_START_DELAY
        # initiate exploration call; single shot to avoid multiple instances
        print('__ starting environment exploration __\n')
        self.explorationTimer=rospy.Timer(rospy.Duration(NODE_START_DELAY),\
            self.explorationTimerCallback,oneshot=True)

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
            .format(self.mapHeight,self.mapWidth,self.mapOrigin.x,self.mapOrigin.y,self.mapOrigin.yaw,self.mapResolution))
        self.mapTiles=self.mapHeight*self.mapWidth
        
    def updateMap(self):
    # update map data
        del self.map# self.map.data.clear
        map=rospy.wait_for_message(self.mapTopic,OccupancyGrid)
        self.map=map.data

    # DATA ############################################################

    def waitCurrentPose(self):
    # wait until /odom is published, then gathers pose data
        odomPose=rospy.wait_for_message(self.odometryTopic,Odometry,timeout=rospy.Duration(20))
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        yaw=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        return coord2D(x,y,yaw)

    def updateReferencePose(self):
    # updates reference positions
        # reference position in WORLD frame
        self.referenceWpoint=self.waitCurrentPose()
        # reference position in MAP frame
        self.referenceMpoint=self.convertToMapPosition(self.referenceWpoint)

    def poseCallback(self,odomPose):
    # updates current position in WORLD frame
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        yaw=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        self.currentWpoint=coord2D(x,y,yaw)


    # EXPLORATION ##########################################################
    
    def explorationTimerCallback(self,event=None):
    # periodic exploration algorithm
        # reference position in WORLD frame and MAP frame
        self.updateReferencePose()
        # look around performing a full rotation in place
        # NEED TO CHECK IS STUCK WHILE ROTATING
        self.doABarrellRoll()
        # find if any frontier point is present by locating them
        if not self.localizeFrontiers():
            # all frontiers explored
            self.explorationCompleted=True
            completionMessage=Bool()
            completionMessage.data=True
            # signals completed exploration message
            # TODO SUBSTITUTE WITH SERVICE TO AVOID HAVING ANOTHER TERMINAL FOR MAPPER FEEDBACK
            self.completionChatter=rospy.Publisher(self.completionTopic,Bool,queue_size=1)#,latch=True)
            self.completionChatter.publish(completionMessage)
            # shows exploration's statistichs
            self.explorationDuration=time.time()-self.explorationDuration
            print('no frontier point found: EXPLORATION COMPLETED in {} seconds\
                ({} iterations)'.format(self.explorationDuration,self.explorationIteration))
            # shuts down exploration timer
            self.explorationTimer.shutdown()
        else:        
            # frontiers to explore still present
            self.explorationIteration+=1
            print('\n----\nITERATION: {}\nreference point {}, {} ({}, {})'.format(self.explorationIteration,\
                self.referenceWpoint.x,self.referenceWpoint.y,self.referenceMpoint.x,self.referenceMpoint.y))
            # computes the next frontier to explore
            self.nextMGoal,self.nextWGoal=self.computeNextGoal()
            # starts goal distance computation
            # self.goalMeter=rospy.Timer(rospy.Duration(0.5),self.goalDistance())
            print('next point to explore: {}, {} ({}, {})\n'.format(\
                    self.nextWGoal.x,self.nextWGoal.y,self.nextMGoal.x,self.nextMGoal.y))
            # computes quantities to select timeout of move_base request
            referenceDistance=self.pointDistance(self.referenceWpoint,self.nextWGoal)
            timeToReachGoal=56+int(1.7*referenceDistance/self.navigator.MAX_LINEAR_VELOCITY)
            # send goal to move_base    
            if self.navigator.requestNewGoal(self.nextWGoal,duration=timeToReachGoal):
                if not self.navigator.robotIsStuck:print('goal successffully reached!')
            else: print('Move_Base cannot reach the goal!')
            # self.goalMeter.shutdown()
            # start a new instance of the timer to perform this callback
            self.explorationTimer=rospy.Timer(rospy.Duration(0.1),self.explorationTimerCallback,oneshot=True)
    
    def localizeFrontiers(self):
    # frontier points identification
        # updates map data
        self.updateMap()
        self.frontiers=[]
        # initializes count of already labelled points
        self.labelledFrontiers=np.ones((self.mapHeight,self.mapWidth))*-1
        # initializes count of map points already explored
        self.chartedTiles=0
        for x in range(self.mapWidth):
            for y in range(self.mapHeight):
                # assesses if a point is already charted
                self.chartedTiles+=self.isCharted(x,y)
                # assesses if a point is a possible frontier
                if self.isEmptySpace(x,y) and self.hasFreeSpaceAround(x,y)\
                  and self.hasUnknownNeighbour(x,y):
                    self.frontiers.append((x,y))
                    self.labelledFrontiers[x][y]=0
        if self.frontiers==[]: return False
        else: return True

    def computeNextGoal(self,mode="empiric"):
        '''ALTERNATIVE: minimize a score to explorer bigger & closer areas
            label frontiers: connected groups
            foreach group: compute centroid
            foreach centroid: send move_base request(service make_plan) to
                if it's reachable: obtain number of poses to reach it
                                    score= number of poses+k/frontiersize
            choosen point= argmin(scores)
            get_plan = rospy.ServiceProxy('/move_base/make_plan', nav_msgs.GetPlan)
            req = nav_msgs.GetPlan()
            req.start = start
            req.goal = Goal
            req.tolerance = .5
            resp = get_plan(req.start, req.goal, req.tolerance)
            print(resp)

            https://python.hotexamples.com/it/examples/nav_msgs.srv/GetPlanRequest/tolerance/python-getplanrequest-tolerance-method-examples.html
            https://programmerall.com/article/87141447221/
        '''
    # compute the next frontier to explore
        # https://jakevdp.github.io/PythonDataScienceHandbook/02.08-sorting.html
        if mode=="empiric":
            self.frontiers.sort(key=\
                lambda x:(x[0]-self.referenceMpoint.x)**2+(x[1]-self.referenceMpoint.y)**2)
            filteredFrontiers=filter(lambda x:(x[0]-self.referenceMpoint.x)**2+\
                (x[1]-self.referenceMpoint.y)**2>int(1/self.mapResolution),self.frontiers)
            if not filteredFrontiers:
                filteredFrontiers=filter(lambda x:(x[0]-self.referenceMpoint.x)**2+\
                    (x[1]-self.referenceMpoint.y)**2>int(0.25/self.mapResolution),self.frontiers)
            self.frontiers=filteredFrontiers
            # choosing a closer objective the more of the map is explored
            chosenIndex=int(len(self.frontiers)/(3+self.selectionCoefficient()))
            nextMGoal=coord2D(self.frontiers[chosenIndex][0],self.frontiers[chosenIndex][1],self.currentWpoint.yaw)   
        # elif mode=="label":
        #     labelledFrontier=self.labelFrontier()
        #     centroids=self.frontierCentroids(labelledFrontier)
        #     scores=self.queryCentroids(centroids)
        #     nextMGoal=self.optimizeScore(scores)         
        nextWGoal=self.convertToWorldPosition(nextMGoal)
        return nextMGoal,nextWGoal
        
    def selectionCoefficient(self):
        ''' incremental coefficient to select the chosen frontier
            starts from a point closer than 66% of other frontiers
            up to 80%
        '''
        chartedPercentage=self.chartedTiles/self.mapTiles
        return 2 if chartedPercentage>0.4 else 2*chartedPercentage
        
    def labelFrontiersTwoPass(self,connected8=False):
    # two pass labelling algorithm
        linked=[]
        labels=[None]*len(self.frontiers)
        nextLabel=0
        # first pass
        for point_x in range(self.mapWidth):
            for point_y in range(self.mapHeight):
                if not self.labelledFrontiers[point_x][point_y]==-1:
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
                # NOTE FOLLOWING LINE WORKS BUT MARKED AS ERROR IN PYTHON 2.7 AND VS CODE
                neighbours=filter(lambda (x,y):x>=0 and x<self.mapHeight and y>=0 and y<self.mapWidth\
                    and not self.labelledFrontiers[point_x][point_y]==-1,neighbours)
                # if neighbours==[] or 
                #     neighboors=

    # def labelFrontier(self):
    #     labelledFrontiers=zip(self.frontiers,[None]*len(self.frontiers))
    #     label=0
    #     for point in labelledFrontiers:
    #         point_label=point[1]
    #         point=point[0]
    #         if point_label==None:
    #             for neigh in labelledFrontiers:
    #                 if not neigh==point and
        
    # MOTION ###############################################################
    
    def doABarrellRoll(self):
    # performs a complete rotation in stationary position
        print('looking around\n')
        rotationMsg=Twist()
        rotationSpeed=self.navigator.MAX_ROTATION_VELOCITY/2
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
        speed=self.navigator.MAX_ROTATION_VELOCITY/2
        velocityMessage.angular.z=speed
        self.velocityChatter.publish(velocityMessage)
        rospy.sleep(2*math.pi*round(random.uniform(0,1),4)/speed)
        # random linear motion
        velocityMessage.angular.z=0
        velocityMessage.linear.x=self.navigator.MAX_LINEAR_VELOCITY*0.8
        self.velocityChatter.publish(velocityMessage)
        rospy.sleep(round(random.uniform(0,3),4))
        velocityMessage.linear.x=0
        self.velocityChatter.publish(velocityMessage)

  
    # FRONTIER POINTS DEFINITION ################################################

    def isEmptySpace(self,point_x,point_y):
    # checks if a poit is empty space
        return self.map[point_y*self.mapWidth+point_x]==0

    def hasFreeSpaceAround(self,point_x,point_y,obstacleMapDistance=0):
    # checks if a possible frontier has obstacles too close
        freespaceCounter=0
        if obstacleMapDistance==0:
            obstacleMapDistance=int(self.OBSTACLE_DISTANCE/self.mapResolution)
        # in a specific square around the candidate point
        for query in range(-obstacleMapDistance,obstacleMapDistance+1):
            query_x=point_x+query
            query_y=point_y+query
            if query_x>0 and query_x<self.mapWidth and query_y>0 and query_y<self.mapHeight:
                if not self.map[query_y*self.mapWidth+query_x]>0: freespaceCounter+=1
        if freespaceCounter>=self.FREESPACE_THRESHOLD: return True
        else: return False

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
        # removes points outside the considered map
        # NOTE FOLLOWING LINE WORKS BUT MARKED AS ERROR IN PYTHON 2.7 AND VS CODE
        neighbours=filter(lambda (x,y):x>=0 and x<self.mapHeight and y>=0 and y<self.mapWidth,neighbours)
        # checks if the point is un-explored
        for neigh in neighbours:
            if self.map[neigh[1]*self.mapWidth+neigh[0]]==-1:
                neighbourCounter+=1

        if (not connected8 and neighbourCounter>=self.UNKNOWN_THRESHOLD)\
        or (connected8 and neighbourCounter>=2*self.UNKNOWN_THRESHOLD): 
            return True
        else: return False


    # UTILITIES ###################################################################

    def isCharted(self,point_x,point_y):
    # checks if a tile has been charted alreasy
        return 0 if self.map[point_y*self.mapWidth+point_x]==-1 else 1

    def convertToMapPosition(self,position):
    # converts coord2D WORLD -> MAP
        return coord2D(int((position.x-self.mapOrigin.x)/self.mapResolution),\
                        int((position.y-self.mapOrigin.y)/self.mapResolution),position.yaw)    
                        
    def convertToWorldPosition(self,position):
    # converts coord2D MAP -> WORLD
        return coord2D(position.x*self.mapResolution+self.mapOrigin.x,\
                        position.y*self.mapResolution+self.mapOrigin.y,position.yaw)

    # def goalDistance(self,event=None):
    #     self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
            
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