#!/usr/bin/env python
from numpy.lib.polynomial import polyint
import rospy
import actionlib
import math
# from actionlib.action_server import ActionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist,PoseStamped
# import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from navigationturt.msg import Setpoint 
# from navigationturt.msg import SetpointAction
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlanRequest,GetPlan,GetPlanResponse
from read_from_txt import extractSingleObjective,readObjectivesFromTextFile
from turtlesim.msg import Pose

class coord2D:
    def __init__(self,x=0,y=0,yaw=0):
        self.x=x
        self.y=y
        self.yaw=yaw


class navigationNode():
    def __init__(self):
        # DEBUG ########
        self.DEBUG=False
        #################
        # W points are referred to WORLD coordiantes
        self.nextWGoal=coord2D()
        self.currentWpoint=coord2D()
        # self.referenceWpoint=coord2D()
        self.previousWpoint=coord2D()
        self.setpointReached=True
        # cinematic parameters
        self.MAX_LINEAR_VELOCITY=0.22#[meters/seconds] of the robot
        self.MAX_ROTATION_VELOCITY=2.84#[rad/s] of the robot {==162.72 deg/s}
        # collision or immovability detection
        self.MOTION_MONITOR_PERIOD=0.8#[seconds] between consecutive firing of motion monitor
        self.collisionCounter=0
        # self.COLLISION_DISTANCE=0.16# motion needed [meters] to consider robot not stuck
        self.COLLISION_DISTANCE=0.5*self.MAX_LINEAR_VELOCITY/self.MOTION_MONITOR_PERIOD
        # self.COLLISION_ANGLE=12# rotation angle [degrees] under which robot is considered stuck
        # NOTE removed since may cause issue when driving with same orientation
        self.MAX_COLLISION_COUNT=18# max allowed iterations count to determine a collision
        self.robotIsStuck=False
        # goal proximity detection
        self.proximityCounter=0
        self.PROXIMITY_DISTANCE=0.6# max distance [meters] to which robot is "close to goal"
        self.MAX_PROXIMITY_COUNT=17# max allowed iterations count to determine proximity to goal
        # map extremes
        self.MIN_X_SETPOINT=-7.39#
        self.MIN_Y_SETPOINT=-6.43#
        self.MAX_X_SETPOINT=7.39#
        self.MAX_Y_SETPOINT=5.17#
        # initialize move_base
        self.MBClient=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.MBClient.wait_for_server()
        self.MBGoal=MoveBaseGoal()
        self.PlanClient=rospy.ServiceProxy('/move_base/make_plan',GetPlan)
        # topics to listen
        self.velocityTopic= rospy.get_param('~velocity_topic', '/cmd_vel')
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')
        self.setpointTopic = rospy.get_param('~setpoint_topic', '/Setpoint')
        self.initializeSetpointListener()
        self.odometryListener=rospy.Subscriber(self.odometryTopic, Odometry,self.odometryCallback, queue_size=5)
        self.velocityCommand=rospy.Publisher(self.velocityTopic,Twist,queue_size=5)
        # PROPORTIONAL REGULATOR #
        self.K_X=0.3
        self.K_Y=0.3
        self.K_CONTROL1=0.35
        self.K_CONTROL2=0.5
        self.LEVER=0.05
        self.MIN_CONTROL_X=0.27
        self.MIN_CONTROL_Y=0
        self.ERROR_THRESHOLD=0.38


    # INITIALIZATION #####################################################

    def loadObjectives(self,targets):
        self.targetsSplitter=extractSingleObjective
        self.totalTargets=len(targets)
        self.targetsServed=0
        self.targets=targets
        print("List of {} objectives received from file,\n deactivating\
             /Setpoint topic listener until it's fully served".format(self.totalTargets))
        self.setpointListener.unregister()
        # launch the specific objective list timer
        self.targetsTimer=rospy.Timer(rospy.Duration(0.1),self.targetsTimerCallback)

    def initializeSetpointListener(self):
        self.setpointListener = rospy.Subscriber(self.setpointTopic,Setpoint,self.setpointCallback,queue_size=5)

    def setNodeParameter(self,param1=None):
    # changes parameter of the robot at runtime
        if not param1==None:self.param1=param1


    # DATA ################################################################

    # def odometryCallback(self,data):

    def odometryCallback(self,odomPose):
    # updates current position in WORLD frame
    ###################
    # REGULATOR VERSION
    #     orientation = data.pose.pose.orientation
    #     position = data.pose.pose.position
    #     siny_cosp = 2*(orientation.w*orientation.z+orientation.x*orientation.y)
    #     cosy_cosp = 1-2*(orientation.y**2 + orientation.z**2)
    #     yaw = math.atan2(siny_cosp,cosy_cosp)
    #     x = position.x + self.LEVER*math.cos(yaw)
    #     y = position.y + self.LEVER*math.sin(yaw)
    ####################
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        yaw=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        self.currentWpoint=coord2D(x,y,yaw)

    def setpointCallback(self,setpointMsg):
    # /Setpoint message callback
        point=coord2D(setpointMsg.x_setpoint,setpointMsg.y_setpoint,setpointMsg.yaw_setpoint)
        self.requestNewGoal(point)

    def waitCurrentPose(self):
    # gathers current pose data after waiting until /odom is published
        odomPose=rospy.wait_for_message(self.odometryTopic,Odometry,timeout=rospy.Duration(20))
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        yaw=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        return coord2D(x,y,yaw)

    def updateReferencePose(self):
    # updates reference position in WORLD frame; initialize point for collision detection
        self.previousWpoint=self.referenceWpoint=self.waitCurrentPose()
        

    # PRIMARY BEHAVIOUR ########################################################
  
    def targetsTimerCallback(self,event=None):
    # timer callback for objectives list
        if self.targetsServed<self.totalTargets:
            self.targetsServed=self.targetsServed+1
            newGoal=self.targetsSplitter(self.targets,self.targetsServed)
            self.requestNewGoal(coord2D(*newGoal))
        elif self.targetsServed==self.totalTargets:
            print('Objectives list fully served!\npublish \
              navigationturt.msg.Setpoint on /Setpoint to request new targets')
            self.targetsTimer.shutdown()
            # starts again listener on /Setpoint
            self.initializeSetpointListener()


# SECONDARY BEHAVIOUR ##########################################################
            
    def motionMonitor(self,event=None):#,straight_recovery=False):
    # monitor correct motion
        self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
        # checks if robot is taking too long to reach the goal from a close position
        proximity_result=self.proximityCheck()
        if proximity_result:
            print('Robot is close enough to the target')
            self.MBClient.cancel_goal()
        # checks if robot is blocked in a given position for too long
        collision_result=self.collisionCheck()
        if collision_result:
            print('robot is stuck near point {}, {}, with orientation {}'\
              .format(self.previousWpoint.x,self.previousWpoint.y,self.previousWpoint.yaw))
            # if not self.robotIsStuck:
            #     self.robotIsStuck=True
            self.monitor.shutdown()
            # self.resetMonitor()
            self.MBClient.cancel_goal()
                # self.randomMotion()
                # self.requestNewGoal(self.referenceWpoint)
                # print('going back to starting point')
        # if not (collision_result or proximity_result):
        #     if not self.robotIsStuck:
        #         self.robotIsStuck=True
                # self.monitor.shutdown()
                # self.resetMonitor()
                # self.MBClient.cancel_goal()
                # self.randomMotion()
                # self.requestNewGoal(self.referenceWpoint)
                # print('going back to starting point')
        
        # self.debugCount+=1
        # if self.debugCount==6:
        #     self.debugCount=0
        #     print('current position {},{}\nreference position {},{}\n previous position {},{}'.\
        #         format(self.currentWpoint.x,self.currentWpoint.y,\
        #             self.referenceWpoint.x,self.referenceWpoint.y\
        #                 ,self.previousWpoint.x,self.previousWpoint.y))

    def resetMonitor(self):
    # resets motion monitor's counters and state
        self.collisionCounter=0
        self.proximityCounter=0
        self.robotIsStuck=False
        self.closeToTarget=False

    # TODO improve this by using cmd_vel signal and position measuring
    def collisionCheck(self):
    # checks if the robot stays close to the same point for too long, hence counts
        if self.pointDistance(self.currentWpoint,self.previousWpoint)<self.COLLISION_DISTANCE:
        # or abs(self.currentWpoint.yaw-self.previousWpoint.yaw)<self.COLLISION_ANGLE:
            self.collisionCounter+=1
            if self.DEBUG:print('+1 collision count: {}'.format(self.collisionCounter))
        else: 
            # if not, start over
            self.robotIsStuck=False
            self.collisionCounter=0
            self.previousWpoint=self.currentWpoint
        # if robot close to target, wait much longer
        if self.collisionCounter>self.MAX_COLLISION_COUNT and not self.closeToTarget\
        or self.collisionCounter>self.MAX_COLLISION_COUNT*2 and self.closeToTarget:
            self.collisionCounter=0
            self.robotIsStuck=True
            return True
        else: return False

    def proximityCheck(self):
    # checks if robot is close, yet not at, the target
        if self.distanceFromGoal<self.PROXIMITY_DISTANCE:
            self.closeToTarget=True
            # if it's close enough, counts
            if self.distanceFromGoal<self.PROXIMITY_DISTANCE*0.42:
                self.proximityCounter+=1
                if self.DEBUG:print('+1 proximity count: {}'.format(self.proximityCounter))
            # otherwise starts over
            else: self.proximityCounter=0
        else: self.closeToTarget=False
        if self.closeToTarget and self.proximityCounter>self.MAX_PROXIMITY_COUNT:
            return True
        else: return False


    # MOTION ##############################################################
    
    def requestNewGoal(self,point,check_plan=True,tolerance=0.2,duration=0,print_message=False):
    # construct move_base goal request
        if self.verifyGoalPoint(point=point,check_plan=check_plan,tolerance=tolerance,\
         duration=duration,print_message=print_message):
            self.updateReferencePose()
            self.MBGoal.target_pose.header.frame_id='map'
            self.MBGoal.target_pose.header.stamp = rospy.Time.now()
            self.MBGoal.target_pose.pose.position.x = self.nextWGoal.x
            self.MBGoal.target_pose.pose.position.y =  self.nextWGoal.y
            self.MBGoal.target_pose.pose.position.z=0
            # self.MBGoal.target_pose.pose.position.x = point.x
            # self.MBGoal.target_pose.pose.position.y =  point.y
            # (qx,qy,qz,qw)=quaternion_from_euler(*(0,0,point.yaw))
            (qx,qy,qz,qw)=quaternion_from_euler(*(0,0,self.nextWGoal.yaw))
            self.MBGoal.target_pose.pose.orientation.x = qx
            self.MBGoal.target_pose.pose.orientation.y = qy
            self.MBGoal.target_pose.pose.orientation.z = qz
            self.MBGoal.target_pose.pose.orientation.w = qw
            #remove obstacles from costmap
            # import std_srvs.srv
            # rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)
            # send the goal to move_base
            self.MBClient.send_goal(self.MBGoal)
            # call motion monitor
            self.resetMonitor()
            self.monitor=rospy.Timer(rospy.Duration(self.MOTION_MONITOR_PERIOD),self.motionMonitor)#straight_recovery))
            requestResult=self.MBClient.wait_for_result() if duration==0\
                    else self.MBClient.wait_for_result(timeout=rospy.Duration(duration))
            self.monitor.shutdown()
            if print_message:
                if requestResult: 
                    if not self.robotIsStuck: print('vvv-- goal reached --vvv\n')
                else: print('xxx-- move base cannot reach the goal --xxx')
            return requestResult

    def verifyGoalPoint(self,point,check_plan=True,tolerance=0.3,duration=0,print_message=False):
    # verifies feasibility of requested goal and 
        point.x=round(point.x,4)
        point.y=round(point.y,4)
        point.yaw=round(point.yaw,4)
        # if check_plan:
        #     start=PoseStamped()
        #     start.pose.position.x=self.currentWpoint.x
        #     start.pose.position.y=self.currentWpoint.y
        #     start.pose.position.z=0
        #     yaw=self.currentWpoint.yaw
        #     qx,qy,qz,qw=quaternion_from_euler(0,0,yaw)
        #     start.pose.orientation.x=qx
        #     start.pose.orientation.y=qy
        #     start.pose.orientation.z=qz
        #     start.pose.orientation.x=qw
        #     goal=PoseStamped()
        #     goal.pose.position.x=point.x
        #     goal.pose.position.y=point.y
        #     goal.pose.position.z=0
        #     yaw=point.yaw
        #     qx,qy,qz,qw=quaternion_from_euler(0,0,yaw)
        #     goal.pose.orientation.x=qx
        #     goal.pose.orientation.y=qy
        #     goal.pose.orientation.z=qz
        #     goal.pose.orientation.x=qw
        #     plan=GetPlanRequest()
        #     plan.start=start
        #     plan.goal=goal
        #     plan.tolerance=tolerance
        #     planResponse=self.PlanClient(plan.start,plan.goal,plan.tolerance)
        #     # if a plan exists, saves the poses to reach goal
        #     # if planResponse: 
        #     #     self.nextGoalPoses=GetPlanResponse()
        #     #     self.nextGoalPoses.plan.poses=???
        # else: planResponse=True
        planResponse=True
        if point.x< self.MIN_X_SETPOINT or\
        point.x>self.MAX_X_SETPOINT or\
        point.y<self.MIN_Y_SETPOINT or\
        point.y>self.MAX_Y_SETPOINT or\
        not planResponse:
            print('received setpoint request\nx: {}\ny: {}\n exceedes limits x: [{} {}]\ny: [{} {}], or no plan found'\
            .format(point.x,point.y,\
                self.MIN_X_SETPOINT,self.MAX_X_SETPOINT,self.MIN_Y_SETPOINT,self.MAX_Y_SETPOINT))
            return False
        else:
            if print_message:
                print('plan found for permitted setpoint x: {}, y: {}, yaw: {}'.\
                format(point.x,point.y,point.yaw))
                if not duration==0: print('timeout set at: {} s'.format(duration))
            self.nextWGoal=coord2D(point.x,point.y,point.yaw)
            return True
        
    def proportionalDecoupled(self,point,print_message=False):
    # tries to reach the goal with proportional regulation
        if self.verifyGoalPoint(point=point,print_message=print_message):
            self.regulatorTimer=rospy.Timer(rospy.Duration(0.2),self.regulatorTimerCallback)
            self.proportionalControlRunning=True
            while self.proportionalControlRunning:pass

    def regulatorTimerCallback(self,event=None):
        # error
        error_x=self.nextWGoal.x-self.currentWpoint.x
        error_y=self.nextWGoal.y-self.currentWpoint.y
        # proportional velocity value
        self.velocity_x=self.K_X*error_x
        self.velocity_y=self.K_Y*error_y
        # proportional action
        self.control1=self.K_CONTROL1*(math.cos(self.currentWpoint.yaw)*self.velocity_x+\
            math.sin(self.currentWpoint.yaw)*self.velocity_y)
        self.control2=self.K_CONTROL2*(-self.velocity_x*(math.sin(self.currentWpoint.yaw/self.LEVER))+\
            self.velocity_y*(math.cos(self.currentWpoint.yaw/self.LEVER)))
        # minimum control value
        # if abs(self.control1)<self.MIN_CONTROL_X:
        #     self.control1=math.copysign(self.MIN_CONTROL_X,self.control1)
        # velocity message
        velocityMessage=Twist()
        velocityMessage.linear.x=self.control1
        velocityMessage.angular.z=self.control2
        self.velocityCommand.publish(velocityMessage)
        if abs(error_x)<self.ERROR_THRESHOLD and\
                abs(error_y)<self.ERROR_THRESHOLD:
                self.proportionalControlRunning=False
                self.regulatorTimer.shutdown()

    def lineTracking(self,point,duration=0,print_message=False):
    # moves along a line connecting current and goal points
        if self.verifyGoalPoint(point=point,print_message=print_message):
            K_h=0.8
            K_d=0.8
            # sloce-intercept form
            delta_x=point.x-self.currentWpoint.x
            delta_y=point.y-self.currentWpoint.y
            slope=delta_y/delta_x
            intercept=point.y-slope*point.x
            # general a*x+b*y+c=0=-m*c+y-p
            a=-slope
            b=1
            c=-intercept
            rate=rospy.Rate(1)
            velocityMessage=Twist()
            distanceTolerance=0.5
            while self.pointDistance(point,self.currentWpoint)>=distanceTolerance:
                # controller1 turns robot toward line to min distance
                distance=(a*self.currentWpoint.x+b*self.currentWpoint.y+c)/math.sqrt(a**2+b**2)
                alfa_d=-K_d*distance
                # controller2 adjusts orientation of robot // to line
                yaw_d=math.atan(-a/b) 
                alfa_h=K_h*(yaw_d-self.currentWpoint.yaw)
                # w/ fixed velocity: gamma=alfa_d+alfa_h
                velocityMessage.linear.x=alfa_h
                velocityMessage.linear.y=0
                velocityMessage.linear.z=0
                velocityMessage.angular.x=0
                velocityMessage.angular.y=0
                velocityMessage.angular.z=alfa_d
                self.velocityCommand.publish(velocityMessage)
                rate.sleep()
            velocityMessage.linear.x=0
            velocityMessage.angular.z=0
            self.velocityCommand.publish(velocityMessage)

    def unicycleMotion(self,point,duration=0,print_message=False):
    # computes angle, turns than drives in straight line toward the point
        if self.verifyGoalPoint(point=point,print_message=print_message):
            distance=self.pointDistance(point,self.currentWpoint)
            error_x=self.nextWGoal.x-self.currentWpoint.x
            error_y=self.nextWGoal.y-self.currentWpoint.y
            orientation=math.atan2(error_y,error_x)
            print('original orientation {}'.format(math.degrees(orientation)))
            print('current yaw {}'.format(math.degrees(self.currentWpoint.yaw)))
            if orientation<self.currentWpoint.yaw:
                orientation-=self.currentWpoint.yaw
            elif orientation>self.currentWpoint.yaw:
                orientation-=self.currentWpoint.yaw
            if abs(orientation)>=2*math.pi:
                 orientation=orientation-math.copysign(2*math.pi,orientation)
            print('final orientation {}'.format(math.degrees(orientation)))
            # rotation
            velocityMessage=Twist()
            velocityMessage.angular.z=math.copysign(self.MAX_ROTATION_VELOCITY,orientation)
            if orientation== self.currentWpoint.yaw:velocityMessage.angular.z=0
            print('if already oriented {}'.format(orientation))
            self.velocityCommand.publish(velocityMessage)
            rospy.sleep(abs(orientation)/self.MAX_ROTATION_VELOCITY)
            # linear motion
            velocityMessage.angular.z=0
            velocityMessage.linear.x=self.MAX_LINEAR_VELOCITY
            self.velocityCommand.publish(velocityMessage)
            rospy.sleep(distance/self.MAX_LINEAR_VELOCITY)
            # stop
            velocityMessage.linear.x=0
            self.velocityCommand.publish(velocityMessage)

    def proportionalSimple(self,point,distance_tolerance=0.6):
        goalPose=Pose()
        goalPose.x=point.x
        goalPose.y=point.y
        distanceTolerance=distance_tolerance
        velocityMessage=Twist()
        COSTANT_LINEAR=0.1
        COSTANT_ANGULAR=0.1
        # COSTANT_ANGULAR=COSTANT_LINEAR
        rate=rospy.Rate(1)
        while self.pointDistance(point,self.currentWpoint)>=distanceTolerance:
            velocityMessage.linear.x=COSTANT_LINEAR*self.pointDistance(point,self.currentWpoint)
            velocityMessage.linear.y=0
            velocityMessage.linear.z=0
            velocityMessage.angular.x=0
            velocityMessage.angular.y=0
            velocityMessage.angular.z=COSTANT_ANGULAR*(math.atan2(point.y-self.currentWpoint.y,point.x-self.currentWpoint.x)-self.currentWpoint.yaw)
            self.velocityCommand.publish(velocityMessage)
            rate.sleep()
        velocityMessage.linear.x=0
        velocityMessage.angular.z=0
        self.velocityCommand.publish(velocityMessage)

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


    # UTILITIES ###################################################################

    # def goalDistance(self,event=None):
    #     self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
            
    def pointDistance(self,pointA,pointB):
        return math.sqrt((pointA.x-pointB.x)**2+(pointA.y-pointB.y)**2)


if __name__ == '__main__':

    print('==#== MOVE BASE ACTIONS BROADCASTER NODE ==#==')
    rospy.init_node('move_base_client',anonymous=True)

    objAddress=rospy.get_param('~objectives_file_location')
    turtleCLient=navigationNode()

    if not objAddress=="":
        objectives=readObjectivesFromTextFile(objAddress)
        turtleCLient.loadObjectives(objectives)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("turtle motion aborted!")