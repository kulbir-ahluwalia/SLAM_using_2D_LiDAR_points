#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 28 00:13:45 2020

@author: andres
"""

import rospy
from nav_msgs.msg import Odometry
import math
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan


#the Publisher is initialized in this line 
pub_odom = rospy.Publisher('odometer_terrasentia', Odometry, queue_size=1)

# pub_odom_gps = rospy.Publisher('odometer_gps', Odometry, queue_size=1)

class listenerNode():
   
    def __init__(self):
        self.loop_hertz = 100.0 #loop frequency
        self.x = 0.0
        self.y = 0.0
        # self.theta = 0.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 0.0

        self.velocity_x = 0.0
        # self.vy = 0.0
        self.yaw_rate = 0.0

        self.orientation = np.array([self.q_x,self.q_y,self.q_z,self.q_w])

        #Following are for laser scan
        self.angle_min = -2.356194496154785
        self.angle_max = 2.356194496154785
        self.angle_increment = 0.004363323096185923

        self.angles_of_lidar = np.linspace(self.angle_min,self.angle_max,1081)
        print("angles from lidar are: ",self.angles_of_lidar)
        self.range_values = np.array([])



        
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop

        rospy.Subscriber("/terrasentia/ekf", Odometry, self.callback)
        rospy.Subscriber("/terrasentia/scan", LaserScan, self.callback2)

        # rospy.Subscriber("Accelx", Float32, self.callback)
        rospy.Subscriber("clock", Float32, self.callback13)

        self.br = tf.TransformBroadcaster()#Initialize the object to be used in the frame transformation
        while not rospy.is_shutdown():
            
            self.linearvelocity()#call the function used to create the Odometry ROS message
            # self.linearvelocity_gps()

            # print("listenerNode.ekf_x is: ",listenerNode.ekf_x)
            # print("listenerNode.ekf_y is: ",listenerNode.ekf_y)

            self.br.sendTransform((self.x, self.y, 0.0), self.orientation, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ

            # self.br.sendTransform((self.gps_x_new, self.gps_y_new, 0.0), self.q_gps, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
            # print(self.x)
            # print("self.odom is: ",self.odom)
            pub_odom.publish(self.odom)            
            self.rate.sleep()
       
        
    def linearvelocity(self):
         self.odom = Odometry()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = self.q_x
        self.odom.pose.pose.orientation.y = self.q_y
        self.odom.pose.pose.orientation.z = self.q_z
        self.odom.pose.pose.orientation.w = self.q_w
        self.odom.twist.twist.linear.x = self.velocity_x
        self.odom.twist.twist.angular.z = self.yaw_rate
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "/map"
        self.odom.child_frame_id = "/base_link"



    def callback(self,msg):
        # listenerNode.ekf_x = msg.pose.pose.position.x
        # listenerNode.ekf_y = msg.pose.pose.position.y
        # listenerNode.ekf_x_velocity = msg.twist.twist.linear.x
        # listenerNode.ekf_z_omega = msg.twist.twist.angular.z

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # msg.pose.pose.orientation.y


        self.q_x = msg.pose.pose.orientation.x
        self.q_y = msg.pose.pose.orientation.y
        self.q_z = msg.pose.pose.orientation.z
        self.q_w = msg.pose.pose.orientation.w

        self.orientation = np.array([self.q_x,self.q_y,self.q_z,self.q_w])


        self.theta = msg.pose.pose.position.y
        self.velocity_x = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        


        # rospy.loginfo("x_accel %s", self.x_accel)

    def callback1(self,msg):
        listenerNode.y_accel = msg.data
        # rospy.loginfo("y_accel %s", self.y_accel)

    def callback2(self,msg):
        # self.angle_min = msg.angle_min
        # self.angle_max = msg.angle_max
        # self.angle_increment = msg.angle_increment

        # print("angle details are: ",self.angle_increment,self.angle_max,self.angle_min)

        self.range_values = np.array(msg.ranges)
        print("type of range_values",type(self.range_values))
        print("length of range_values",len(self.range_values))
        print("range_values numpy array is:",self.range_values)

        #since angle increment and min and max angle does not change, we set the values

        self.x_value_lidar = np.cos(self.angles_of_lidar)*self.range_values
        self.y_value_lidar = np.sin(self.angles_of_lidar)*self.range_values

        print("x values from lidar are:", self.x_value_lidar)
        print("y values from lidar are:", self.y_value_lidar)

        plt.plot(self.y_value_lidar,self.x_value_lidar)
        plt.grid(True)
        # plt.show()



 
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    
    rospy.init_node('LiDAR_example_node', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()

