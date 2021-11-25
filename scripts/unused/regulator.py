#!/usr/bin/env python

from actionlib.action_server import ActionServer
import rospy
from navigationturt.msg import Setpoint 
from navigationturt.msg import SetpointAction
from geometry_msgs.msg import Twist#, Vector3
from nav_msgs.msg import Odometry
import actionlib
import math


class regulatorNode():
    def __init__(self):
        self.VEL_ANGULAR_MAX=0.3
        self.VEL_LINEAR_MAX=0.3
        self.VEL_ANGULAR_MIN=0.1
        self.VEL_LINEAR_MIN=0.1
        self.lever=0.05
        self.MIN_CONTROL_X=0.3
        self.ERROR_THRESHOLD=0.1

        # self.controlMode='PROPORTIONAL'
        self.controlMode=rospy.get_param('/control_mode','PROPORTIONAL')

        self.KX=0.2
        self.KY=0.2
        self.control1=0
        self.control2=0
        self.KControl1=1
        self.KControl2=1

        self.setpointX=0
        self.setpointY=0
        self.setpointReached=True

        self.currentPositionX=0
        self.currentPositionY=0
        self.currentOrientation=0
        self.velocityX=0
        self.velocityY=0

        self.robotStuck=False
        self.previousPositionX=0
        self.previousPositionY=0
        self.lastMotilePositionX=0
        self.lastMotilePositionY=0
        self.setpointBackup=(0,0)

        self.velocityTopic= rospy.get_param('~velocity_topic', '/cmd_vel')
        self.setpointTopic = rospy.get_param('~setpoint_topic', '/Setpoint')
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')

        # setpoint action server
        self.setpointServer=actionlib.SimpleActionServer('setpoint_server',SetpointAction,\
                                                        self.actionCallback,False)
        self.setpointServer.start()
        #always start the server when ready to process goals
        #use wait_for_server() in the client to exploit this

        # setpoint subscriber
        self.setpointListener = rospy.Subscriber(self.setpointTopic,Setpoint,\
                                                 self.setpointCallback,queue_size=5)
        # odometry subscriber
        self.odometryListener=rospy.Subscriber(self.odometryTopic, Odometry,\
                                                self.odometryCallback, queue_size=5)
        
        # velocity publisher
        self.velocityCommand=rospy.Publisher(self.velocityTopic,Twist,queue_size=5)
        self.velocityMessage=Twist()

        #timer
        self.regulatorTimer=rospy.Timer(rospy.Duration(0.01),self.regulatorCallback)

    def actionCallback(self,goal):
        print("debug")
        self.setpointServer.set_succeeded()


    def setpointCallback(self,data):
        self.setpointX=round(data.x_setpoint,4)
        self.setpointY=round(data.y_setpoint,4)
        self.setpointReached=False
        print("received a new goal position: x="+str(self.setpointX)+
                                            " ,y="+str(self.setpointY))

    
    def odometryCallback(self,data):
        orientation = data.pose.pose.orientation
        position = data.pose.pose.position
        siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
        cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
        self.currentOrientation = math.atan2(siny_cosp,cosy_cosp)
        self.currentPositionX = position.x + self.lever*math.cos(self.currentOrientation)
        self.currentPositionY = position.y + self.lever*math.sin(self.currentOrientation)
    

    def regulatorCallback(self,data):

        if not self.setpointReached:

            positionErrorX=self.setpointX-self.currentPositionX
            positionErrorY=self.setpointY-self.currentPositionY

            if self.controlMode=='PROPORTIONAL':

                self.velocityX=self.KX*positionErrorX
                self.velocityY=self.KY*positionErrorY

                self.control1=self.KControl1*(math.cos(self.currentOrientation)*self.velocityX+\
                    math.sin(self.currentOrientation)*self.velocityY)
                self.control2=self.KControl2*(-self.velocityX*(math.sin(self.currentOrientation/self.lever))+\
                    self.velocityY*(math.cos(self.currentOrientation/self.lever)))
            
                # minimum speed
                if abs(self.control1)<self.MIN_CONTROL_X:
                    self.control1=math.copysign(self.MIN_CONTROL_X,self.control1)
            
                self.velocityMessage.linear.x=self.control1
                self.velocityMessage.angular.z=self.control2
                self.velocityCommand.publish(self.velocityMessage)
            
                if abs(positionErrorX)<self.ERROR_THRESHOLD and\
                        abs(positionErrorY)<self.ERROR_THRESHOLD:
                    self.setpointReached=True
                    print('Setpoint Reached')
            
            else:
                self.goalDistance=math.sqrt(positionErrorX**2+positionErrorY**2)
                self.goalOrientation=math.atan2(positionErrorY,positionErrorX)
                # print("error "+str(self.positionErrorX)+str(self.positionErrorY))

                # rotation
                self.velocityMessage.angular.z=self.VEL_ANGULAR_MAX
                self.velocityCommand.publish(self.velocityMessage)
                rospy.sleep(self.goalOrientation/self.VEL_ANGULAR_MAX)
                # linear motion
                self.velocityMessage.angular.z=0
                self.velocityMessage.linear.x=self.VEL_LINEAR_MAX
                self.velocityCommand.publish(self.velocityMessage)
                rospy.sleep(self.goalDistance/self.VEL_LINEAR_MAX)
                # stop
                self.velocityMessage.linear.x=0
                self.velocityCommand.publish(self.velocityMessage)
                self.setpointReached=True
                print('Setpoint Reached')

            if self.robotStuck:
                self.setpointReached=False
                (self.setpointX,self.setpointY)=self.setpointBackup


if __name__ == '__main__':
    initialMessage = rospy.get_param('~message', '------starting turtlebot regulator------')
    print(initialMessage)

    # nodeName=rospy.get_param('~node_name', 'turtlebot_regulator')
    # rospy.get_name()#only after init_node
    
    rospy.init_node(rospy.get_param\
                ('~node_name', 'turtlebot_regulator'),anonymous=True)

    turtleRegulator=regulatorNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("turtle motion aborted!")