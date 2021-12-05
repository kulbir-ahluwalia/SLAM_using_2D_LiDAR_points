#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-

#subscribe to terrasentia/scan and convert from range,theta to x,y 
# then we do line regression



# msg is {'callerid': '/play_1638658591733923465', 'latching': '0', 'md5sum': 'cd5e73d190d741a2f92e81eda573aca7', 
# 'message_definition': "# This represents an estimate of a position and velocity in free space.  \n
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n
# # The twist in this message should be specified in the coordinate frame given by the child_frame_id\n
# Header 
# header\n
# string child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n\n
# 
# ================================================================================\n
# 
# MSG: std_msgs/Header\n
# # Standard metadata for higher-level stamped data types.\n
# # This is generally used to communicate timestamped data \n
# # in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n
# #Two-integer timestamp that is expressed as:\n
# # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n
# # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n
# # time-handling sugar is provided by the client library\ntime stamp\n
# #Frame this data is associated with\nstring frame_id\n
# \n================================================================================\n
# MSG: geometry_msgs/PoseWithCovariance\n
# 
# # This represents a pose in free space with uncertainty.\n\n
# Pose pose\n\n
# # Row-major representation of the 6x6 covariance matrix\n
# # The orientation parameters use a fixed-axis representation.\n
# # In order, the parameters are:\n
# # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n
# float64[36] covariance\n
# 
# \n================================================================================\n
# MSG: geometry_msgs/Pose\n
# # A representation of pose in free space, composed of position and orientation. \n
# Point position\nQuaternion orientation\n
# 
# \n================================================================================\n
# MSG: geometry_msgs/Point\n# This contains the position of a point in free space\n
# float64 x
# \nfloat64 y
# \nfloat64 z
# 
# \n\n================================================================================\n
# MSG: geometry_msgs/Quaternion\n
# # This represents an orientation in free space in quaternion form.\n\n
# float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n\n
# ================================================================================\n
# MSG: geometry_msgs/TwistWithCovariance\n
# # This expresses velocity in free space with uncertainty.\n\n
# Twist twist\n\n# Row-major representation of the 6x6 covariance matrix\n
# # The orientation parameters use a fixed-axis representation.\n
# # In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n
# float64[36] covariance\n\n
# ================================================================================\n
# MSG: geometry_msgs/Twist\n# This expresses velocity in free space broken into its linear and angular parts.\n
# Vector3  linear\n
# Vector3  angular
# \n\n================================================================================\n
# MSG: geometry_msgs/Vector3\n# This represents a vector in free space. \n
# # It is only meant to represent a direction. Therefore, it does not\n
# # make sense to apply a translation to it (e.g., when applying a \n
# # generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n
# # geometry_msgs/Point message instead.\n\n
# float64 x
# \nfloat64 y\n
# float64 z\n
# ", 'topic': '/terrasentia/ekf', 'type': 'nav_msgs/Odometry'}

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_from_euler

class listenerNode():
      
    ekf_x = 0.0
    ekf_y = 0.0



    # A=0.0
    def __init__(self):
        self.loop_hertz = 100.0
        self.x = 0.0
        self.y = 0.0
        
    
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop
        #callback is called whenever something is published on the topic /ekf
        rospy.Subscriber("/terrasentia/ekf", Odometry, self.callback)

        while not rospy.is_shutdown():
            #it's update rate is affected by self.loop_hertz
            #rostopic hz /terrasentia/ekf is about 50Hz on average
            #so keep it at 100 hz for no lag
            print("listenerNode.ekf_x is: ",listenerNode.ekf_x)
            print("listenerNode.ekf_y is: ",listenerNode.ekf_y)
            self.rate.sleep()

   
    def callback(self,msg):
        #does not matter if it is self.x or listenerNode.ekf_x since there is only one class and one instance
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation
        print("self.orientation is : ",self.orientation)
        print("self.orientation is : ",type(self.orientation))

        self.q = quaternion_from_euler(0.0, 0.0, 45)
        print("self.q is : ",self.q)
        print("self.q is : ",type(self.q))


        #print info about the message:
        # print("msg is",msg._connection_header)

        # print("msg is",msg)
        # print("header is: ",msg.header)
        print("msg.pose.pose.position.x is:",self.x)
        print("msg.pose.pose.position.y is:",self.y)
        


        # rospy.loginfo("msg is %s", msg)
        
if __name__ == '__main__':
    #Initialize the node and name it
    rospy.init_node('rospy_subscriber', anonymous = True)
    ne = listenerNode()
    ne.run()


