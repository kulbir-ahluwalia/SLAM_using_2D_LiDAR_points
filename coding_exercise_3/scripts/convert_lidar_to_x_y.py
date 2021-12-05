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
    
    #accelerometer
    x_accel = 0.0
    y_accel = 0.0
    z_accel = 0.0

    # ekf_x = 0.0
    # ekf_y = 0.0
    # ekf_x_velocity = 0.0
    # ekf_z_omega = 0.0

    # #velocity 
    # speed_bl = 0.0
    # speed_br = 0.0
    # speed_fl = 0.0
    # speed_fr = 0.0

    # #gyroscope
    # pitch_gyro = 0.0
    # roll_gyro = 0.0
    # yaw_gyro = 0.0

    # timestamp_var = 0.0

    # #gps
    # latitude_var = 0.0
    # longitute_var = 0.0

    # x_before_gps = 0.0
    # y_before_gps = 0.0

    clock = 0.0

    

    


   
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
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        self.range_values = np.array([])




        # self.velocity_x_gps = 0.0
        # self.gps_delta_x = 0.0
        # self.gps_delta_y = 0.0
        # self.theta_gps = 0.0
        # self.dt = 0.025
        # self.yaw_rate_gps = 0.0

        # self.gps_x = 0.0
        # self.gps_y = 0.0
        # self.x_before_gps = 0.0
        # self.y_before_gps = 0.0

        # self.gps_x_new = 0.0
        # self.gps_y_new = 0.0

        # self.gps_x_rviz = 0.0
        # self.gps_y_rviz = 0.0

        # self.flag = 0.0

        # self.yaw_rate = (self.yaw_gyro - self.theta)/self.dt



        
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop

        rospy.Subscriber("/terrasentia/ekf", Odometry, self.callback)
        rospy.Subscriber("/terrasentia/scan", LaserScan, self.callback2)

        # rospy.Subscriber("Accelx", Float32, self.callback)
        # rospy.Subscriber("Accely", Float32, self.callback1)
        # rospy.Subscriber("Accelz", Float32, self.callback2)

        # rospy.Subscriber("Blspeed", Float32, self.callback3)
        # rospy.Subscriber("Brspeed", Float32, self.callback4)
        # rospy.Subscriber("Flspeed", Float32, self.callback5)
        # rospy.Subscriber("Frspeed", Float32, self.callback6)

        # rospy.Subscriber("Gyro_pitch", Float32, self.callback7)
        # rospy.Subscriber("Gyro_roll", Float32, self.callback8)
        # rospy.Subscriber("Gyro_yaw", Float32, self.callback9)

        # rospy.Subscriber("Timestamp", Float32, self.callback10)

        # rospy.Subscriber("latitude", Float32, self.callback11)
        # rospy.Subscriber("longitude", Float32, self.callback12)

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
            # pub_odom_gps.publish(self.odometer_gps)
            
############Replace this part with the x and y positions determined using the information from the rosbag##################
            # if self.x < 10.0 and self.y == 0.0:
            #     self.x+=0.5
            #     self.y == 0.0
            #     if self.x == 10.0:
            #         self.theta = -math.pi/2
        
            # elif self.x == 10.0 and self.y == 0.0:
            #     self.y -= 0.5
            #     self.theta = -math.pi/2
            # elif self.x == 10.0 and self.y < 0 and self.y > -10.0:
            #     self.y -= 0.5
            #     self.x = 10.0
            #     if self.y == -10.0:
            #         self.theta -=math.pi/2
            # elif self.x > 0.0 and self.y == -10.0:
            #     self.x-=0.5
            #     self.y = -10.0
            #     if self.x == 0.0:
            #         self.theta -=math.pi/2
            # elif self.x == 0.0 and self.y < 0.0:
            #     self.y += 0.5
            #     self.x = 0.0
            #     if self.y == 0.0:
            #         self.theta -=math.pi/2
                
###########################################################################################################################
            self.rate.sleep()
       
        
    def linearvelocity(self):

        #calculate the speed using average of 4 wheels
        # self.velocity_x = (self.speed_bl + self.speed_br + self.speed_fl + self.speed_fr)*0.25
        # print("Self.velocity is: ",self.velocity_x)


        # if self.velocity_x is not None:
            
            # self.theta = self.theta + (self.yaw_rate*self.dt)
            # self.theta = self.theta + (self.yaw_gyro*self.dt)
            # self.x = self.x + self.velocity_x*math.cos(self.theta)*self.dt
            # self.y = self.y + self.velocity_x*math.sin(self.theta)*self.dt

            
        # self.q = quaternion_from_euler(0.0, 0.0, self.theta)#function used to convert euler angles to quaternions

        

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
        msg.pose.pose.orientation.y


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
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

        print("angle details are: ",self.angle_increment,self.angle_max,self.angle_min)

        self.range_values = np.array(msg.ranges)
        print("type of range_values",type(self.range_values))
        print("length of range_values",len(self.range_values))
        print("range_values numpy array is:",self.range_values)


        # rospy.loginfo("z_accel %s", self.z_accel)



    def callback3(self,msg):
        listenerNode.speed_bl = msg.data
        # rospy.loginfo("speed_bl %s", self.speed_bl)

    def callback4(self,msg):
        listenerNode.speed_br = msg.data
        # rospy.loginfo("speed_br %s", self.speed_br)

    def callback5(self,msg):
        listenerNode.speed_fl = msg.data
        # rospy.loginfo("speed_fl %s", self.speed_fl)

    def callback6(self,msg):
        listenerNode.speed_fr = msg.data
        # rospy.loginfo("speed_fr %s", self.speed_fr)




    def callback7(self,msg):
        listenerNode.pitch_gyro = msg.data
        # rospy.loginfo("pitch_gyro %s", self.pitch_gyro)

    def callback8(self,msg):
        listenerNode.roll_gyro = msg.data
        # rospy.loginfo("roll_gyro %s", self.roll_gyro)

    def callback9(self,msg):
        listenerNode.yaw_gyro = msg.data
        # rospy.loginfo("yaw_gyro %s", self.yaw_gyro)



    def callback10(self,msg):
        listenerNode.timestamp_var = msg.data
        # rospy.loginfo("timestamp_var %s", self.timestamp_var)


    def callback11(self,msg):
        listenerNode.latitude_var = msg.data
        # rospy.loginfo("latitude_var %s", self.latitude_var)

    def callback12(self,msg):
        listenerNode.longitute_var = msg.data
        # rospy.loginfo("longitute_var %s", self.longitute_var)

    def callback13(self,msg):
        listenerNode.clock = msg.data
        # rospy.loginfo("clock %s", self.clock)

    # def callback14(self,msg):
    #     listenerNode.x_accel = msg.data
    #     rospy.loginfo("number %s", self.x_accel)

 
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    
    rospy.init_node('LiDAR_example_node', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()

