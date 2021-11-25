#!/usr/bin/env python

from numpy.core.fromnumeric import resize, shape
import rospy
import actionlib
import math
import random
import time
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal#,MoveBaseActionFeedback
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener,TransformStamped
from action_move_base import coord2D, navigationNode
# from regulator import regulatorNode
import numpy as np

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

class point2D:
    def __init__(self,x=0,y=0):
        self.x=x
        self.y=y

class room2D:
    def __init__(self,map_height,map_width,map_resolution=None,method='limits',\
        lower_x=None,upper_x=None,left_y=None,right_y=None,object=None):
        ''' 
    selected room definition:
        by limits: calculates the points considering vertical orientation;
                    must be already in MAP coordinate frame
        by points: extrapolate limits from points, any orientation
        by object: directly defines the room's boolean mask
        '''
        self.roomMask=np.zeros((map_height,map_width),dtype=bool)
        if method=='limits':
            min_x=lower_x if lower_x<upper_x else upper_x
            max_x=lower_x if lower_x>upper_x else upper_x
            min_y=left_y if left_y<right_y else right_y
            max_y=left_y if left_y>right_y else right_y
            self.size_x=abs(min_x-max_x)
            self.size_y=abs(min_y-max_y)
            # excludes points within a certain distance from room borders
            if map_resolution!=None:
                self.goalMask=np.zeros((map_height,map_width),dtype=bool)
                wall_threshold=1
                wall_threshold=int(wall_threshold/map_resolution)
                self.goalMask[min_y+wall_threshold-1:max_y-wall_threshold,\
                    min_x-1+wall_threshold:max_x-wall_threshold]=True
            # self.ULcorner=point2D(upper_x,left_y)
            # self.URcorner=point2D(upper_x,right_y)
            # self.LLcorner=point2D(lower_x,left_y)
            # self.LRcorner=point2D(lower_x,right_y)
            # self.startingPoint=coord2D((lower_x+upper_x)/2,(left_y+right_y)/2)
            self.roomMask[min_y-1:max_y,min_x-1:max_x]=True


###################################################################################

class cleaningNode:
    def __init__(self):
        #### DEBUG ######
        self.counter=0
        #################
        self.THRESHOLD_DISTANCE=0.1#distance [meters] until which robot's encumbrance block uvs
        self.UV_POWER=100E-6#power [watt/meter^2] of UV lamp
        self.REQUIRED_ENERGY=10E-3# [joule] for each tile
        self.REQUIRED_ENERGY_GRAPHICAL=127# represents self.REQUIRED_ENERGY graphically in rviz
        self.GRAPHICAL_RATIO=self.REQUIRED_ENERGY_GRAPHICAL/self.REQUIRED_ENERGY
        self.AMOUNT_THRESHOLD=3#[%] of points that can be left out of cleaning
        self.totalToBeCleaned=[]# will store the total amount of points to be cleaned in the room
        # topics to listen
        self.laserTopic=rospy.get_param('~laserscan_topic','/scan')
        self.odometryTopic= rospy.get_param('~odometry_topic','/odom')# trying to use navigator one
        self.UVmapTopic= rospy.get_param('~uv_map_topic','/uv_map')
        self.mapTopic= rospy.get_param('~map_topic','/map')
        self.velocityTopic= rospy.get_param('~velocity_topic','/cmd_vel')
        self.costMapTopic=rospy.get_param('~cost_map_topic','/move_base/global_costmap/costmap')
        # self.MAP_FRAME= rospy.get_param('~map_frame','map')
        # self.LASER_FRAME= rospy.get_param('~laser_frame','scan')
        # initialize map
        MAP_RESOLUTION=0.10#[meters]
        self.initializeMap(MAP_RESOLUTION)
        # uv map publisher
        self.mapChatter=rospy.Publisher(self.UVmapTopic,OccupancyGrid,queue_size=5)
        # initializes scan
        self.getLaserData()
        self.laserListener=rospy.Subscriber(self.laserTopic,LaserScan,self.laserCallback,queue_size=5)
        # define the selected room
        # ul: -0.098, 5.07
        # ll:-4.829, 5.1
        # ur: -0.145, -0.041
        # lr: -4.987, -0.046
        lower_x,upper_x,left_y,right_y=-4.89,-0.12,-0.04,5.09
        # optional WORLD point to start the cleaning operation
        # startingPoint=coord2D((lower_x+upper_x)/2,(left_y+right_y)/2)
        lower_x=self.convertPToMapPosition(lower_x)#25
        upper_x=self.convertPToMapPosition(upper_x)+1#49
        left_y=self.convertPToMapPosition(left_y)#49
        right_y=self.convertPToMapPosition(right_y)+1#75
        self.limits=[lower_x,upper_x,left_y,right_y]
        self.selectedRoom=room2D(map_height=self.mapHeight,map_width=self.mapWidth,\
            #,map_resolution=self.mapResolution 
        lower_x=lower_x,upper_x=upper_x,left_y=left_y,right_y=right_y)
        # sets all points outside the room of interest to already enough energy
        self.globalUVamount=np.ones(shape(self.globalUVamount))*~self.selectedRoom.roomMask*self.REQUIRED_ENERGY#_GRAPHICAL
        # initiate cleaning call; single shot to avoid multiple instances
        NODE_START_DELAY=rospy.get_param('node_start_delay',default=3.0)
        self.cleaningTimer=rospy.Timer(rospy.Duration(NODE_START_DELAY),\
            self.cleaningTimerCallback,oneshot=True)
        # initialize navigation client
        self.navigator=navigationNode()
        self.navigator.DEBUG=False
        # adapt navigator params for this task
        self.navigator.PROXIMITY_DISTANCE=4*self.navigator.PROXIMITY_DISTANCE
        self.navigator.COLLISION_DISTANCE=0.8*self.navigator.COLLISION_DISTANCE
        self.navigator.MAX_PROXIMITY_COUNT=round(0.45*self.navigator.MAX_PROXIMITY_COUNT)
        self.navigator.MAX_COLLISION_COUNT=round(0.45*self.navigator.MAX_COLLISION_COUNT)
        # colelcts cost map data
        # self.initializeCostMap()
        # awaits cleaning operation start
        rospy.wait_for_message(self.UVmapTopic,OccupancyGrid)
        print('---- cleaning operation started: BEWARE UV RADIATION ----\n\t---\t---\t\t\t---\t---\n')
        # move the robot to initial position inside the room
        # startingPoint=coord2D(x=-4.5,y=4.5)
        # self.navigator.requestNewGoal(startingPoint)
        # initiate navigation call
        self.unreachablePoints=[]
        # self.EXECUTE_RECOVERY_BEHAVIOUR=True
        self.EXECUTE_RECOVERY_BEHAVIOUR=False
        self.operationTimer=rospy.Timer(rospy.Duration(NODE_START_DELAY),\
            self.operationTimerCallback,oneshot=True)
        
    
    # DATA #######################################################

    def initializeMap(self,mapResolution=None):
    # uses gathered map data to initiate uv global map
        map=rospy.wait_for_message(self.mapTopic,OccupancyGrid,timeout=rospy.Duration(20))
        self.map=map.data
        self.globalUVmap=OccupancyGrid()
        self.globalUVmap.header.frame_id='odom'
        externaResolution=round(map.info.resolution,4)
        # checks if a user defined value has been passed
        self.mapResolution=(externaResolution if mapResolution==None else mapResolution)
        self.mapHeight=self.globalUVmap.info.height=int(map.info.height*externaResolution/self.mapResolution)
        self.mapWidth=self.globalUVmap.info.width=int(map.info.width*externaResolution/self.mapResolution)
        self.globalUVmap.info.origin.position.x=map.info.origin.position.x
        self.globalUVmap.info.origin.position.y=map.info.origin.position.y
        self.globalUVmap.info.origin.position.z=map.info.origin.position.z
        self.globalUVmap.info.origin.orientation.x=map.info.origin.orientation.x
        self.globalUVmap.info.origin.orientation.y=map.info.origin.orientation.y
        self.globalUVmap.info.origin.orientation.z=map.info.origin.orientation.z
        self.globalUVmap.info.origin.orientation.w=map.info.origin.orientation.w
        self.mapOrigin=coord2D(map.info.origin.position.x,map.info.origin.position.y,\
            euler_from_quaternion((map.info.origin.orientation.x,map.info.origin.orientation.y,\
                            map.info.origin.orientation.z,map.info.origin.orientation.w))[2])
        self.globalUVmap.info.resolution=self.mapResolution
        # initialize the map containing the total energy absorbed by each point
        self.globalUVamount = np.zeros((self.mapHeight,self.mapWidth))

    # def initializeCostMap(self):
    # # gathers global cost map data
    #     map=rospy.wait_for_message(self.costMapTopic,OccupancyGrid,timeout=rospy.Duration(20))
    #     self.costMap=map.data
    #     # NOTE any height,width,origin and resolution coincide with (native) environment's one

    def getLaserData(self):
    # requests laser scan information
        laser=rospy.wait_for_message(self.laserTopic,LaserScan,timeout=rospy.Duration(20))
        self.SCAN_ANGLE_MIN=laser.angle_min#0
        self.SCAN_ANGLE_MAX=laser.angle_max#2pi [rad]
        self.SCAN_ANGLE_INCREMENT=laser.angle_increment#0.0175 [rad]==1 [deg]
        self.SCAN_RANGE_MIN=laser.range_min# 0.12 [m]
        self.SCAN_RANGE_MAX=laser.range_max# 3.5 [m]
        self.SCAN_ARRAY_SIZE=len(laser.ranges)# 360
        self.LOCAL_GRID_WIDTH=2*laser.range_max/self.mapResolution# in grid points
        self.LOCAL_GRID_HEIGHT=2*laser.range_max/self.mapResolution# in grid points
        print('#\t#\t#\nscan DATA:\nangle min: {}\tangle max: {}\nangle increment: {}\nrange min: {}\nrange max: {}\narray size: {}\n#\t#\t#'\
            .format(self.SCAN_ANGLE_MIN,self.SCAN_ANGLE_MAX,self.SCAN_ANGLE_INCREMENT,self.SCAN_RANGE_MIN,self.SCAN_RANGE_MAX,self.SCAN_ARRAY_SIZE))
        
    def laserCallback(self,laserData):
    # updates scan readings
        self.laserRanges=laserData.ranges

    def poseCallback(self,odomPose):
    # updates current position
        x=round(odomPose.pose.pose.position.x,4)
        y=round(odomPose.pose.pose.position.y,4)
        yaw=round(euler_from_quaternion((odomPose.pose.pose.orientation.x,odomPose.pose.pose.orientation.y,\
                odomPose.pose.pose.orientation.z,odomPose.pose.pose.orientation.w))[2],4)
        # current WORLD point from odometry
        self.currentWpoint=coord2D(x,y,yaw)
        self.currentMpoint=self.convertToMapPosition(self.currentWpoint)
        
    def publishUVmap(self):
    # updates published information about total amount of received energy
        self.globalUVmap.data=np.zeros((self.mapHeight*self.mapWidth),dtype=np.uint8)
        # flattens the matrix to a list and imposes a ceiling value
        self.globalUVmap.data= [round(cell*self.GRAPHICAL_RATIO) if round(self.GRAPHICAL_RATIO*cell)<self.REQUIRED_ENERGY_GRAPHICAL\
         else self.REQUIRED_ENERGY_GRAPHICAL for row in self.globalUVamount for cell in row]
        self.globalUVmap.info.map_load_time=rospy.Time.now()
        self.mapChatter.publish(self.globalUVmap)

    # def showPosition(self,event=None):
    #     print('current position {},{}\nreference position {},{}\n previous position {},{}'.\
    #         format(self.navigator.currentWpoint.x,self.navigator.currentWpoint.y,\
    #             self.navigator.referenceWpoint.x,self.navigator.referenceWpoint.y\
    #                 ,self.navigator.previousWpoint.x,self.navigator.previousWpoint.y))

    # MOTION #######################################################

    def operationTimerCallback(self,event=None):
    # motion routine
        # computes and requests next goal
        nextCleaningPoint=self.selectNextCleaningPoint()#far_point=True)
        duration=round(0.12*(max(self.selectedRoom.size_x,self.selectedRoom.size_y)/\
            self.navigator.MAX_LINEAR_VELOCITY))
        request=self.navigator.requestNewGoal(nextCleaningPoint,duration=duration,print_message=True)
        if not request or self.navigator.robotIsStuck:
            # adds point to the list of unreachable
            x=nextCleaningPoint.x
            y=nextCleaningPoint.y
            unreachable_point=y,x
            self.unreachablePoints.append(unreachable_point)
            if self.EXECUTE_RECOVERY_BEHAVIOUR:
            # recovery behaviour
                recovery_method='proportional'
                if recovery_method=='decoupled':
                    # starts decoupled proportiona controller
                    print('trying decoupled proportional recovery behaviour')
                    self.navigator.proportionalDecoupled(point=nextCleaningPoint)
                elif recovery_method=='unicycle':
                    # starts unicycle driving behaviour
                    print('trying unicycle recovery behaviour')
                    self.navigator.unicycleMotion(nextCleaningPoint)
                elif recovery_method=='proportional':
                    # starts simple proportional controller
                    print('trying simple proportional recovery behaviour')
                    self.navigator.proportionalSimple(nextCleaningPoint)
                elif recovery_method=='line':
                    # starts line tracking proportional controller
                    print('trying line proportional recovery behaviour')
                    self.navigator.lineTracking(nextCleaningPoint)
                elif recovery_method=='farpoint':
                    # reaching the farest point
                    print('reaching a far point recovery behaviour')
                    nextCleaningPoint=self.selectNextCleaningPoint(far_point=True)
                    self.navigator.requestNewGoal(nextCleaningPoint,duration=duration,print_message=True)
                print('')
        elif request:
            # wait few seconds to clean the area
            area_cleaning_duration=4#[seconds]
            print('cleaning this area for {} seconds\n'.format(area_cleaning_duration))
            rospy.sleep(area_cleaning_duration)
        # checks if cleaning operation is terminated;
        if not self.roomIsCleaned():
            # if not: calls a new instance of this routine
            self.operationTimer=rospy.Timer(rospy.Duration(0.05),\
              self.operationTimerCallback,oneshot=True)

    def roomIsCleaned(self):
    # checks if all points in selected room received enough energy
        # status= np.min(self.globalUVamount[self.selectedRoom.roomMask])==self.REQUIRED_ENERGY
        # checks if amounts of points to be cleaned is below a specidic threshold
        status=int(100*self.amountToBeCleaned/self.totalToBeCleaned)<self.AMOUNT_THRESHOLD
        if status:
            self.operationTimer.shutdown()
            self.cleaningTimer.shutdown()
        return status
          
    def selectNextCleaningPoint(self,far_point=False):
        '''selects the best point to clean with respect to:
                distance, point and neighbourhood energy received'''
        # extract points which didn't received enough energy yet
        points=self.pointsRequiringEnergy()
        # removes points that cannot be reached
        temp=self.permittedPoints(points)
        if temp!=[]:points=temp
        scores=[]
        for x,y in points:
        # for y,x in points:
            # compute distances from current robot position
            distance=self.pointDistance(self.convertToWorldPosition(coord2D(x,y)),self.navigator.currentWpoint)
            # compute point's cost on global occupancy map
            # global_cost=self.costMap[y*self.mapWidth+x]
            # scores.append([x,y,distance,global_cost])
            scores.append([x,y,distance])
        scores=np.array(scores)
        # sort by increasing distance
        scores=scores[scores[:,2].argsort()]
        # considering only points outside the robot's shadow or a defined threshold
        threshold_distance=5*self.THRESHOLD_DISTANCE
        filteredScores=scores[scores[:,2]>threshold_distance]
        scores=filteredScores if not np.any(filteredScores) else scores
        if far_point:
            # returns farest point; usend when robot is stuck
            return self.convertToWorldPosition(coord2D(scores[-1,1],scores[-1,0]))
        # neighbourhood edge size in MAP points; must be odd
        neigh_size=5
        # compute local and average (in a defined neighbourhood) energy received by points
        neigh_energies=[]
        energies=[]
        for x,y in scores[:,:2]:
            energies.append(self.globalUVamount[int(x),int(y)])
            neigh_energies.append(np.average(self.globalUVamount[\
              int(x-(neigh_size**2-1)/2):int(x+(neigh_size**2-1)/2),int(y-(neigh_size**2-1)/2):int(y+(neigh_size**2-1)/2)]))
        # concatenates columns to score array
        scores=np.c_[scores,np.array(energies),np.array(neigh_energies)]
        # computes and minimize the score to find the next goal
        results=self.computeScore(scores)
        scores=np.c_[scores, results]
        scores=scores[scores[:,-1].argsort()]
        return self.convertToWorldPosition(coord2D(scores[0,1],scores[0,0]))

    def pointsRequiringEnergy(self):
    # returns MAP coordinates of points with less than required UV amount
        result=[]
        mask=np.where(self.selectedRoom.roomMask)
        mask= zip(mask[0],mask[1])
        indeces=(np.where(self.selectedRoom.roomMask*self.globalUVamount<self.REQUIRED_ENERGY))
        indeces= zip(indeces[0],indeces[1])# format: y(row), x(col)
        for index in indeces:
            # excludes points not in the room
            if index in mask:result.append(index)
        # amount of points still requiring energy
        self.amountToBeCleaned=len(result)
        # initializes the total amount of points to be cleaned
        if self.totalToBeCleaned==[]:self.totalToBeCleaned=self.amountToBeCleaned
        # print('percentage of cleaned points {}'.format(int(100*(1-self.amountToBeCleaned/self.totalToBeCleaned))))
        # TODO NOT WORKING
        return result

    def permittedPoints(self,points):
    # check if any of the points requiring energy must be avoided
        # print('remaining tiles to be cleaned {}'.format(self.amountToBeCleaned))
        # removes points alreasy known to be unreachable
        remotion_method=''
        if remotion_method=='comprehension':
            result=[point for point in points if not point in self.unreachablePoints]
        elif remotion_method=='filter':
            result=filter(lambda point: point not in self.unreachablePoints,points)
        elif remotion_method=='remove':
            for point in self.unreachablePoints:
                try: points.remove(point);result=points
                except ValueError:pass
        elif remotion_method=='set':
            result=list(set(points)-set(self.unreachablePoints))
        else: result=points
        # self.amountToBeCleaned=len(result)
        return result

    # def pointsFarFromObstacles(self,points):#,radius=0):
    # # TODO NOT WORKING
    # # discards points inside a given radious from an obstacle
    #     # predefined distance from obstacles
    #     # if radius==0: radius=2
    #     # map_radius=int(radius/self.mapResolution)
    #     # results=[]
    #     # counter=0
    #     # for point in points:
    #     #     counter+=1
    #     #     point_x=point[1]
    #     #     point_y=point[0]
    #     #     # checks inside the defined MAP distance
    #     #     for query in range(-map_radius,map_radius+1):
    #     #         query_x=point_x+query
    #     #         query_y=point_y+query
    #     #         if query_x>0 and query_x<self.mapWidth and query_y>0 and query_y<self.mapHeight:
    #     #             if not self.map[query_y*self.mapWidth+query_x]>0:
    #     #                 if not point in results:results.append(point)
    #     mask=np.where(self.selectedRoom.goalMask)       
    #     mask=zip(mask[0],mask[1])
    #     results=[]
    #     for point in points:
    #         if point in mask: results.append(point)
    #     return results

    def computeScore(self,scores):
    # score is increasing with distance, cost map value and energy
        distances=scores[:,[2]]
        ave_dist=np.average(distances)
        distances=distances/ave_dist
        # coeff_dist=0.67
        # discount_dist=0.55
        discount_dist=self.adaptiveDistanceDiscount()
        # cost_dist=0
        # global_costs=scores[:,3]
        # ave_glob=np.average(global_costs)
        # global_costs=global_costs/ave_glob
        # discount_glob=0.67
        locals=scores[:,[-2]]
        ave_loc=np.average(locals)
        locals=locals/ave_loc
        # discount_loc=0.35
        discount_loc=0
        neighboorhood=scores[:,[-1]]
        ave_nei=np.average(neighboorhood)
        neighboorhood=neighboorhood/ave_nei
        discount_nei=0.55
        # coeff_nei=0
        results=np.zeros((len(scores),1))#+rand_comp
        if not discount_dist==0: results=results+distances/discount_dist
        # if not discount_glob==0: results=results+global_costs/discount_glob
        if not discount_loc==0: results=results+locals/discount_loc
        if not discount_nei==0: results=results+neighboorhood/discount_nei
        rand_comp=3.6*np.random.rand(len(scores),1)+np.ones((len(scores),1))
        results=[a*b for a,b in zip(results,rand_comp)]
        return results

    def adaptiveDistanceDiscount(self,base_value=0.55,ratio=0.9):
    # returns increasing discount the more the unreachable points
        return base_value+ratio*len(self.unreachablePoints)

    # CLEANING ###################################################

    def cleaningTimerCallback(self,event=None):
    # normal operation callback for UV cleaning
        # time_now=time.time()
        self.calculateUV()
        # print(time.time()-time_now)
        self.publishUVmap()
        self.cleaningTimer=rospy.Timer(rospy.Duration(0.05),self.cleaningTimerCallback,oneshot=True)

    def calculateUV(self):#,room,approximation='round'):
    # calculates instantaneous UV radiation for each reachable point
        currentMpoint=self.convertToMapPosition(self.navigator.currentWpoint)
        # robot encumbrance projected shadow computation in MAP frame
        robotMencumbrance=int(self.THRESHOLD_DISTANCE/self.mapResolution)
        servedPoints=[]
        # compute reached points using scan values; ranges in tangential direction
        for angle_index in range(self.SCAN_ARRAY_SIZE):
            angle= angle_index*self.SCAN_ANGLE_INCREMENT+currentMpoint.yaw
            if angle>2*math.pi: angle-=2*math.pi
            # if range is infinite, considers scan max range (irradiance power drops <10%); otherwise the sensed one
            try:
                if not math.isinf(self.laserRanges[angle_index]): coveredDistance= int(self.laserRanges[angle_index]/self.mapResolution)
                else: coveredDistance=int(self.SCAN_RANGE_MAX/self.mapResolution)
            except Exception: pass
            # ranges in radial direction
            for distance in range(robotMencumbrance,coveredDistance+1):
                # truncate approximation
                point_x=int(currentMpoint.x+distance*math.cos(angle))
                point_y=int(currentMpoint.y+distance*math.sin(angle))
                pointM=(point_x,point_y)
                # checks if point's irradiation already been computed for this call
                if not pointM in servedPoints:
                    servedPoints.append(pointM)
                    pointW=self.convertToWorldPosition(coord2D(pointM[0],pointM[1]))
                    irradiation=self.calculateIrradiation(pointW)
                    # sum to previous total
                    self.globalUVamount[pointM[1],pointM[0]]+=irradiation
        
    def calculateIrradiation(self,point):
    # returns power reaching that point
        distance=self.pointDistance(self.navigator.currentWpoint,point)
        return self.UV_POWER/distance**2 if distance>=self.THRESHOLD_DISTANCE else 0
                        

    # UTILITIES ################################################################

    def convertToMapPosition(self,position):
    # converts coord2D WORLD -> MAP  
        return coord2D(int((position.x-self.mapOrigin.x)/self.mapResolution),\
                int((position.y-self.mapOrigin.y)/self.mapResolution),position.yaw)  

    def convertPToMapPosition(self,position,axis=None):
    # converts coord2D WORLD -> MAP  
        if axis==None or axis=='x':
            return int((position-self.mapOrigin.x)/self.mapResolution)
        if axis=='y':
            return int((position-self.mapOrigin.y)/self.mapResolution) 

    def convertToWorldPosition(self,position):
    # converts coord2D MAP -> WORLD    
        return coord2D(position.x*self.mapResolution+self.mapOrigin.x,\
                            position.y*self.mapResolution+self.mapOrigin.y,position.yaw)

    def convertPToWorldPosition(self,position,axis=None):
    # converts coord2D MAP -> WORLD    
        if axis==None or axis=='x':
            return position*self.mapResolution+self.mapOrigin.x
        if axis=='y':
            return position*self.mapResolution+self.mapOrigin.y

    # def goalDistance(self,event=None):
    #     self.distanceFromGoal=self.pointDistance(self.currentWpoint,self.nextWGoal)
            
    def pointDistance(self,pointA,pointB):
    # returns distance between two points
        return math.sqrt((pointA.x-pointB.x)**2+(pointA.y-pointB.y)**2)

    def printPosition(self):
    # prints current position in MAP frame
        Mpoint=self.convertToMapPosition(self.navigator.currentWpoint)
        print(Mpoint.x,Mpoint.y,Mpoint.yaw)

###################################################################################

if __name__ == '__main__':
    print('==#== UV CLEANING NODE ==#==\n')
    rospy.init_node('cleaner',anonymous=True)
    cleaner=cleaningNode()
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("cleaning operation aborted!")
