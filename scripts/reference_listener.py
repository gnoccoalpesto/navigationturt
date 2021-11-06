#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg


'''
class TFStructure:
    def __init__(self):
        self.translation_x=float
        self.translation_y=float
        self.translation_z=float
        self.rotation_x=float
        self.rotation_y=float
        self.rotation_z=float
        self.rotation_w=float
        self.frame_id=str
        self.child_id=str
        self.static_transform=bool
'''

class TFListener:
    def __init__(self,child,father,use_lookup,direct_transform):
        
        self.DIRECT_TRANSFORM=direct_transform
        self.LISTEN_WITH_LOOKUP=use_lookup
        self.messageRead=False

        if self.LISTEN_WITH_LOOKUP:
            self.TFBuffer = tf2_ros.Buffer()
            self.Listener = tf2_ros.TransformListener(self.TFBuffer)
        else:
            self.Listener=rospy.Subscriber('/tf',tf2_msgs.msg.TFMessage,\
                                            self.TFCallback,queue_size=1)

        # self.nodeName=rospy.get_name()
        self.target_frame_id =child
        self.starting_frame_id =father
        # self.target_frame_id = rospy.get_param('~frame_id','%s_father_link'%self.nodeName)
        # self.starting_frame_id = rospy.get_param('~child_frame_id','%s_child_link'%self.nodeName)
        # self.target_frame_id = 'imu_link'
        # self.starting_frame_id = 'scan_link'
        print('child frame: '+child)
        print('father frame: '+father)

        # self.listenerTimer=rospy.timer(rospy.Duration(0.1), self.listenTF)

        if self.LISTEN_WITH_LOOKUP:
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                try:
                    self.transform = self.TFBuffer.lookup_transform(\
                        self.target_frame_id, self.starting_frame_id, rospy.Time())
                except  (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                            tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue

                self.messageRead=True
                rate.sleep()

    def TFCallback(self,data):
        for data_content in data.transforms:
            if data_content.child_frame_id==self.target_frame_id and (\
                (self.DIRECT_TRANSFORM\
                and data_content.header.frame_id==self.starting_frame_id)\
            or not self.DIRECT_TRANSFORM ):
                self.transform=data_content.transform
                self.messageRead=True
    '''
    def listenTF(self,event=None):

        try:
            self.transformation = self.TFBuffer.lookup_transform(self.target_frame_id, self.starting_frame_id, rospy.Time())
            print(self.transformation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
                        tf2_ros.ExtrapolationException):
            print('failed')
            # exit(1)
    '''
        
    # def __del__(self):

if __name__ == '__main__':

    rospy.init_node('tf2_listener')

    lis=TFListener(\
        child='imu_link',father='scan_link',use_lookup=False,direct_transform=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        # del lis
        print('bye')


