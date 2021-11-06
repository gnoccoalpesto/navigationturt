#!/usr/bin/env python

import rospy
import actionlib
import math
import random
import time
# from actionlib.action_server import ActionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import std_srvs.srv
# import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from navigationturt.msg import Setpoint 
# from navigationturt.msg import SetpointAction
from nav_msgs.msg import Odometry
from read_from_txt import extractSingleObjective,readObjectivesFromTextFile


class coord2D:
    def __init__(self,x=0,y=0,teta=0):
        self.x=x
        self.y=y
        self.teta=teta


class moveBaseClient():
    def __init__(self,targets=None):
      
        self.MIN_X_SETPOINT=-7.39#
        self.MIN_Y_SETPOINT=-6.43#
        self.MAX_X_SETPOINT=7.39#
        self.MAX_Y_SETPOINT=5.17#
        self.setpointX=0
        self.setpointY=0
        self.setpointTeta=0
        self.nextWGoal=coord2D()
        self.currentX=0
        self.currentY=0
        self.currentTeta=0 
        self.currentWpoint=coord2D()

        self.newGoalReceived=bool()
        self.setpointReached=True
        
        # initialize move_base
        self.MBClient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.MBClient.wait_for_server()
        self.MBGoal=MoveBaseGoal()

        self.robotIsStuck=False
        self.collisionCounter=0
        self.proximityCounter=0
        self.MAX_LINEAR_VELOCITY=0.22#[meters/seconds] of the robot
        self.MAX_ROTATION_VELOCITY=2.84#[rad/s] of the robot {==162.72 deg/s}
        self.COLLISION_DISTANCE=0.16# motion needed [meters] to consider robot not stuck
        self.COLLISION_ANGLE=12# rotation angle [degrees] under which robot is considered stuck
        self.MAX_COLLISION_COUNT=18# max allowed iterations count to determine a collision
        self.PROXIMITY_DISTANCE=0.6# max distance [meters] to which robot is "close to goal"
        self.MAX_PROXIMITY_COUNT=17# max allowed iterations count to determine proximity to goal

        if not targets==None:
            print('using .txt file')
            self.targetsSplitter=extractSingleObjective
            self.totalTargets=len(targets)
            self.targetsServed=0
            self.targets=targets
            self.targetsTimer=rospy.Timer(rospy.Duration(0.1),self.targetsTimerCallback)
        else: print('==#== waiting navigationturt.msg.Setpoint on /Setpoint ==#==')

        # topics to listen
        self.velocityTopic= rospy.get_param('~velocity_topic', '/cmd_vel')
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')
        self.setpointTopic = rospy.get_param('~setpoint_topic', '/Setpoint')
        NODE_START_DELAY=rospy.get_param('node_start_delay',default=3.0)

        self.setpointListener = rospy.Subscriber(self.setpointTopic,Setpoint,self.setpointCallback,queue_size=5)
        self.odometryListener=rospy.Subscriber(self.odometryTopic, Odometry,self.odometryCallback, queue_size=5)
        self.velocityChatter=rospy.Publisher(self.velocityTopic,Twist,queue_size=1)# velocity broadcaster for full rotation

    # INITIALIZATION #####################################################

    def loadObjectivesFromFile(self,targets):
        print('using .txt file')
        self.targetsSplitter=extractSingleObjective
        self.totalTargets=len(targets)
        self.targetsServed=0
        self.targets=targets
        self.targetsTimer=rospy.Timer(rospy.Duration(0.1),self.targetsTimerCallback)

    def initializeSetpointListener(self):
        self.setpointListener = rospy.Subscriber(self.setpointTopic,Setpoint,\
                                                 self.setpointCallback,queue_size=5)

    # def initializeRequestListener(self):


    # DATA ################################################################

    def odometryCallback(self,odomPose):
    # updates current position
        self.currentX=odomPose.pose.pose.position.x
        x=round(odomPose.pose.pose.position.x,4)
        self.currentY=odomPose.pose.pose.position.y
        y=round(odomPose.pose.pose.position.y,4)
        self.currentTeta=euler_from_quaternion((odomPose.pose.pose.orientation.x,\
            odomPose.pose.pose.orientation.y,odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2]
        teta=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        # current WORLD point from odometry
        self.currentWpoint=coord2D(x,y,teta)

    def setpointCallback(self,data):
        # self.newGoalReceived=True
        if self.setpointReached:
            self.setpointFunction((data.x_setpoint,data.y_setpoint,data.teta_setpoint))

    def goalRequestCallback(self,data):
    # open a new direct goal request
        self.newGoalReceived=True
        self.nextGoal=data

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
        

    # SECONDARY BEHAVIOUR ##########################################################

    # intervention in case robot cannot really reach the target but it gets "close" to it
    def proximityTimerCallback(self,event=None):
        if ((self.currentX-self.setpointX)**2+\
        (self.currentY-self.setpointY)**2)<self.PROXIMITY_DISTANCE:
            self.proximityCounter=self.proximityCounter+1
        
        if self.proximityCounter>25:
            print('ROBOT IS CLOSE TO TARGET BUT CANNOT REACH IT...cancelling!')
            self.proximityCounter=0
            self.setpointReached=True
            self.MBClient.cancel_goal()
            self.proximityTimer.shutdown()
            rospy.sleep(0.5)
            
    # def motionMonitor(self,event=None):
    # # monitor correct motion
    #     self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
    #     # checks if robot is taking too long to reach the goal from a close position
    #     if self.proximityCheck(): self.MBClient.cancel_goal()
    #     # checks if robot is blocked in a given position for too long
    #     if self.collisionCheck():# print('collision')
    #         if not self.robotIsStuck:
    #             self.monitor.shutdown()
    #             self.resetMonitor()
    #             # self.MBClient.cancel_goal()
    #             # self.randomMotion()
    #             # self.requestNewGoal(self.referenceWpoint)
    #             # print('going back to starting point')
    #             self.robotIsStuck=True

    # def resetMonitor(self):
    # # resets motion monitor's counters and state
    #     self.collisionCounter=0
    #     self.proximityCounter=0
    #     self.robotIsStuck=False
    #     self.closeToTarget=False

    # def collisionCheck(self):
    # # checks if the robot stays close to the same point for too long, hence counts
    #     if self.pointDistance(self.currentWpoint,self.previousWpoint)<self.COLLISION_DISTANCE\
    #     or abs(self.currentWpoint.teta-self.previousWpoint.teta)<self.COLLISION_ANGLE:
    #         self.collisionCounter+=1
    #     else: 
    #         # if not, start over
    #         self.robotIsStuck=False
    #         self.collisionCounter=0
    #         self.previousWpoint=self.currentWpoint
    #     # if robot close to target, wait much longer
    #     if self.collisionCounter>self.MAX_COLLISION_COUNT and not self.closeToTarget\
    #     or self.collisionCounter>self.MAX_COLLISION_COUNT*2 and self.closeToTarget:
    #         self.collisionCounter=0
    #         print('robot is stuck near point {}, {}, with orientation {}'\
    #             .format(self.previousWpoint.x,self.previousWpoint.y,self.previousWpoint.teta))
    #         return True
    #     else: return False

    # def proximityCheck(self):
    # # checks if robot is close, yet not at, the target
    #     if self.distanceFromGoal<self.GOAL_DISTANCE:
    #         self.closeToTarget=True
    #         # if it's close enough, counts
    #         if self.distanceFromGoal<self.GOAL_DISTANCE*0.42:
    #             self.proximityCounter+=1
    #         # otherwise starts over
    #         else: self.proximityCounter=0
    #     else: self.closeToTarget=False
    #     if self.closeToTarget and self.proximityCounter>self.MAX_PROXIMITY_COUNT:
    #         print('Robot is close enough to the target\n')
    #         return True
    #     else: return False

    # PRIMARY BEHAVIOUR ########################################################À
  
    def targetsTimerCallback(self,event=None):
        if self.targetsServed<self.totalTargets and self.setpointReached:
            self.targetsServed=self.targetsServed+1

            newSetpoint=self.targetsSplitter(self.targets,self.targetsServed)
            self.setpointFunction(newSetpoint)
   
        elif self.targetsServed==self.totalTargets:
            print('all targets have been reached')
            print('publish navigationturt.msg.Setpoint on /Setpoint to request new targets')
            self.targetsTimer.shutdown()

    # MOTION ##############################################################
    
    def setpointFunction(self,newSetpoint):
    # def requestNewGoal(self,point,rpy=(0,0,0),duration=0):
    # # construct move_base goal request
        self.setpointReached=False
        self.setpointX=round(newSetpoint[0],4)
        self.setpointY=round(newSetpoint[1],4)
        self.setpointTeta=round(newSetpoint[2],4)

        if self.setpointX< self.MIN_X_SETPOINT or\
        self.setpointX>self.MAX_X_SETPOINT or\
        self.setpointY<self.MIN_Y_SETPOINT or\
        self.setpointY>self.MAX_Y_SETPOINT:
            print('received setpoint request\nx: {}\ny: {}\n exceedes limits x: [{} {}]\ny: [{} {}]'\
            .format(self.setpointX,self.setpointY,\
                self.MIN_X_SETPOINT,self.MAX_X_SETPOINT,self.MIN_Y_SETPOINT,self.MAX_Y_SETPOINT))
            self.setpointReached=True
        else:
            print('permitted setpoint request received\nx: {}\ny: {}\nteta: {}'.\
                format(self.setpointX,self.setpointY,self.setpointTeta))

            self.MBGoal.target_pose.header.frame_id='map'
            self.MBGoal.target_pose.header.stamp = rospy.Time.now()
            self.MBGoal.target_pose.pose.position.x = self.setpointX
            self.MBGoal.target_pose.pose.position.y =  self.setpointY
            #self.MBGoal.target_pose.pose.position.x=point.x
            #self.MBGoal.target_pose.pose.position.y=point.y
            #self.MBGoal.target_pose.pose.position.z=0
            q = quaternion_from_euler(0, 0, self.setpointTeta)
            self.MBGoal.target_pose.pose.orientation.x = q[0]
            self.MBGoal.target_pose.pose.orientation.y = q[1]
            self.MBGoal.target_pose.pose.orientation.z = q[2]
            self.MBGoal.target_pose.pose.orientation.w = q[3]
            #(qx,qy,qz,qw)=quaternion_from_euler(*rpy)
            # self.MBGoal.target_pose.pose.orientation.x=qx
            # self.MBGoal.target_pose.pose.orientation.y=qy
            # self.MBGoal.target_pose.pose.orientation.z=qz
            # self.MBGoal.target_pose.pose.orientation.w=qw
            rospy.sleep(2)

            #remove obstacles from costmap
            # rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)

            self.MBClient.send_goal(self.MBGoal)
            #     referenceDistance=self.pointDistance(self.referenceWpoint,self.nextWGoal)
            #     timeToReachGoal=56+int(1.7*referenceDistance/self.MAX_LINEAR_VELOCITY)
            #     # call motion monitor
            #     self.monitor=rospy.Timer(rospy.Duration(0.8),self.motionMonitor)
            self.proximityTimer=rospy.Timer(rospy.Duration(1),self.proximityTimerCallback)
            self.waiter=self.MBClient.wait_for_result()

            #     requestResult=self.MBClient.wait_for_result() if duration==0\
            #          else self.MBClient.wait_for_result(rospy.Duration(duration))

            if not self.waiter:
                print('error')
            else:
                print('goal successfully reached!')
                self.actionResult=self.MBClient.get_result()
                # rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)
                print(self.actionResult)
            
            self.setpointReached=True
            self.proximityTimer.shutdown()
            #     self.monitor.shutdown()
            #     self.resetMonitor()
            #     return requestResult
        
    # def randomMotion(self):
    # # performs a random rotation followed by a random linear motion
    #     print('performing a motion in random direction\n')
    #     velocityMessage=Twist()
    #     # random rotation
    #     speed=self.MAX_ROTATION_VELOCITY/2
    #     velocityMessage.angular.z=speed
    #     self.velocityChatter.publish(velocityMessage)
    #     rospy.sleep(2*math.pi*round(random.uniform(0,1),4)/speed)
    #     # random linear motion
    #     velocityMessage.angular.z=0
    #     velocityMessage.linear.x=self.MAX_LINEAR_VELOCITY*0.8
    #     self.velocityChatter.publish(velocityMessage)
    #     rospy.sleep(round(random.uniform(0,3),4))
    #     velocityMessage.linear.x=0
    #     self.velocityChatter.publish(velocityMessage)

    # def doABarrellRoll(self):
    # # performs a complete rotation in stationary position
    #     print('looking around\n')
    #     rotationMsg=Twist()
    #     rotationSpeed=self.MAX_ROTATION_VELOCITY/2
    #     rotationMsg.angular.z=rotationSpeed
    #     self.velocityChatter.publish(rotationMsg)
    #     rospy.sleep(2*math.pi/rotationSpeed)
    #     rotationMsg.angular.z=0
    #     self.velocityChatter.publish(rotationMsg)


    # UTILITIES ###################################################################

    # def convertToMapPosition(self,position):
    # # converts coord2D WORLD -> MAP
    #     return coord2D(int((position.x-self.mapOrigin.x)/self.mapResolution),\
    #                     int((position.y-self.mapOrigin.y)/self.mapResolution),position.teta)    
                        
    # def convertToWorldPosition(self,position):
    # # converts coord2D MAP -> WORLD
    #     return coord2D(position.x*self.mapResolution+self.mapOrigin.x,\
    #                     position.y*self.mapResolution+self.mapOrigin.y,position.teta)

    # def goalDistance(self,event=None):
    #     self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
            
    def pointDistance(self,pointA,pointB):
        return math.sqrt((pointA.x-pointB.x)**2+(pointA.y-pointB.y)**2)


if __name__ == '__main__':

    print('==#== MOVE BASE ACTIONS BROADCASTER NODE ==#==')
    rospy.init_node('move_base_client',anonymous=True)

    objAddress=rospy.get_param('~objectives_file_location')

    # MODIFY BY STANDARD INIT THEN CALLING IT WITH FILENAME, LISTENER OR DIRECT REQUEST
    if objAddress=="":
        turtleCLient=moveBaseClient()
    else:
        objectives=readObjectivesFromTextFile(objAddress)
        turtleCLient=moveBaseClient(objectives)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("turtle motion aborted!")