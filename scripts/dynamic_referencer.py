#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

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


class dynamicTFBroadcaster:

    def __init__(self):

        self.nodeName=rospy.get_name()

        self.trasformIsStatic=True#bool(rospy.get_param('~is_static','true'))
        BROADCAST_WITH_LOOKUP=True#rospy.get_param('~use_lookup_transform','False')

        if not BROADCAST_WITH_LOOKUP:
            # print("publisher broadcast")
            self.TFBroacaster = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        else:
            # print("lookup broadcast")
            if self.trasformIsStatic:
                self.TFBroadcaster  = tf2_ros.StaticTransformBroadcaster()
            else:
                self.TFBroadcaster=tf2_ros.TransformBroadcaster()

        self.TFMessage = geometry_msgs.msg.TransformStamped()

        # self.broadcasterTimer=rospy.timer(rospy.Duration(0.1), self.TFPublisher)

    # def TFPublisher(self,TFData):
        self.TFMessage.header.frame_id = rospy.get_param('~frame_id','%s_father_link'%self.nodeName)
        self.TFMessage.child_frame_id = rospy.get_param('~child_frame_id','%s_child_link'%self.nodeName)
        self.TFMessage.transform.translation.x = float(rospy.get_param('~translation_x','0'))
        self.TFMessage.transform.translation.y = float(rospy.get_param('~translation_y','0'))
        self.TFMessage.transform.translation.z = float(rospy.get_param('~translation_z','0'))
        rotation_r=float(rospy.get_param('~rotation_r','0'))
        rotation_p=float(rospy.get_param('~rotation_p','0'))
        rotation_y=float(rospy.get_param('~rotation_y','0'))
        quaternionTF=quaternion_from_euler(rotation_r,rotation_p,rotation_y)
        self.TFMessage.transform.rotation.x = quaternionTF[0]
        self.TFMessage.transform.rotation.y =  quaternionTF[1]
        self.TFMessage.transform.rotation.z =  quaternionTF[2]
        self.TFMessage.transform.rotation.w =  quaternionTF[3]

        while not rospy.is_shutdown():
            self.TFMessage.header.stamp = rospy.Time.now()
        
            if not BROADCAST_WITH_LOOKUP:
                self.TFBroacaster.publish([self.TFMessage])
            else:
                self.TFBroadcaster.sendTransform(self.TFMessage)

            rospy.sleep(0.1)

            # quaternions basics
            #https://wiki.ros.org/tf2/Tutorials/Quaternions

    def __del__(self):
        rospy.delete_param('~frame_id')
        rospy.delete_param('~child_frame_id')
        rospy.delete_param('~translation_x')
        rospy.delete_param('~translation_y')
        rospy.delete_param('~translation_z')
        rospy.delete_param('~rotation_x')
        rospy.delete_param('~rotation_y')
        rospy.delete_param('~rotation_z')
        rospy.delete_param('~rotation_w')


if __name__ == '__main__':

    rospy.init_node('referencer',anonymous=True)
    
    br=dynamicTFBroadcaster()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        del br