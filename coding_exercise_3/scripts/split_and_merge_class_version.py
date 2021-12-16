#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tf
import numpy as np
import math as m
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from sklearn import linear_model, datasets


# the Publisher is initialized in this line
pub_odom = rospy.Publisher('odometer_terrasentia', Odometry, queue_size=1)

# pub_odom_gps = rospy.Publisher('odometer_gps', Odometry, queue_size=1)


class listenerNode():

    def __init__(self):
        self.loop_hertz = 100.0  # loop frequency
        self.x = 0.0
        self.y = 0.0
        # self.theta = 0.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 0.0
        self.marker_id_number = 1
        self.right_xy_array = []
        self.left_xy_array = []

        self.velocity_x = 0.0
        # self.vy = 0.0
        self.yaw_rate = 0.0

        self.orientation = np.array([self.q_x, self.q_y, self.q_z, self.q_w])

        # Following are for laser scan
        self.angle_min = -2.356194496154785
        self.angle_max = 2.356194496154785
        self.angle_increment = 0.004363323096185923

        self.angles_of_lidar = np.linspace(self.angle_min, self.angle_max, 1081)
        # print("angles from lidar are: ", self.angles_of_lidar)
        self.range_values = np.array([])
        self.marker_2_new = None

    def run(self):
        # this line is used to declare the time loop
        self.rate = rospy.Rate(self.loop_hertz)

        rospy.Subscriber("/terrasentia/ekf", Odometry, self.callback_terrasentia_ekf)
        rospy.Subscriber("/terrasentia/scan", LaserScan, self.callback_laser_scan)

        # rospy.Subscriber("/terrasentia/scan", LaserScan, callback)

        # rospy.Subscriber("Accelx", Float32, self.callback)
        # rospy.Subscriber("clock", Float32, self.callback13)

        # Initialize the object to be used in the frame transformation
        self.br = tf.TransformBroadcaster()

        while not rospy.is_shutdown():

            if len(self.left_xy_array)>0:

                self.linearvelocity()  # call the function used to create the Odometry ROS message
                # self.linearvelocity_gps()

                # print("listenerNode.ekf_x is: ",listenerNode.ekf_x)
                # print("listenerNode.ekf_y is: ",listenerNode.ekf_y)

                # this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
                # self.br.sendTransform(
                #     # (self.x, self.y, 0.0), self.orientation, rospy.Time.now(), "/base_link", "/map")
                #     (self.x, self.y, 0.0), self.orientation, rospy.Time.now(), "/camera_link", "/map")

                # self.br.sendTransform((self.gps_x_new, self.gps_y_new, 0.0), self.q_gps, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
                # print(self.x)
                # print("self.odom is: ",self.odom)
                self.marker_2_new = self.LiDAR_MARKER_2()
                self.split_and_merge_algorithm_custom(self.right_xy_array, Threshold_distance)
                self.split_and_merge_algorithm_custom(self.left_xy_array, Threshold_distance)
                pub_odom.publish(self.odom)
                pub_rviz.publish(self.marker_2_new)
                # self.rate.sleep()

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

    def callback(self, msg):
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

        self.orientation = np.array([self.q_x, self.q_y, self.q_z, self.q_w])

        self.theta = msg.pose.pose.position.y
        self.velocity_x = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    # def publish_cluster_of_points_to_marker(self,points):
    #     marker = self.MakeMarker(points)
    #     pub_rviz.publish(marker)

    def callback_laser_scan(self, msg):
        print("Laser scan callback function called")
        # self.angle_min = msg.angle_min
        # self.angle_max = msg.angle_max
        # self.angle_increment = msg.angle_increment

        # print("angle details are: ",self.angle_increment,self.angle_max,self.angle_min)

        self.range_values = np.array(msg.ranges)
        # print("type of range_values", type(self.range_values))
        # print("length of range_values", len(self.range_values))
        # print("range_values numpy array is:", self.range_values)

        # since angle increment and min and max angle does not change, we set the values

        # self.x_value_lidar = np.cos(self.angles_of_lidar)*self.range_values
        # self.y_value_lidar = np.sin(self.angles_of_lidar)*self.range_values

        # print("x values from lidar are:", self.x_value_lidar)
        # print("y values from lidar are:", self.y_value_lidar)

        # plt.plot(self.y_value_lidar, self.x_value_lidar)
        # plt.grid(True)

        global pub_rviz
        global Threshold_distance
        ranges = np.asarray(msg.ranges)
        alpha = np.linspace(msg.angle_min, msg.angle_max, 1081)
        self.right_xy_array,self.left_xy_array = self.polar_coord_to_cartesian_coord_fn(ranges, alpha)
        
    

        
    #we also filter the distances here, we consider only the distances which are within [0.01999,30]
    def polar_coord_to_cartesian_coord_fn(self,r, alpha):
        filtered_r = []
        filtered_alpha = []
        for i in range(0,len(r)):
            # if r[i]>=0.01999 and r[i]<30:
            if r[i]>=0.02 and r[i]<30:
                # print("r is: ",r[i])
                filtered_r.append(r[i])
                filtered_alpha.append(alpha[i])

        
        # print("filtered_r is:",filtered_r)
        # print("shape of filtered r is:", filtered_r_numpy.shape)

        # print("filtered_alpha is:",filtered_alpha)
        # print("shape of filtered alpha is:", filtered_alpha_numpy.shape)

        left_xy = []
        right_xy = []

        #create left and right divisions

        for i in range(0,len(filtered_alpha)):
            x = filtered_r[i]*np.cos(filtered_alpha[i])
            y = filtered_r[i]*np.sin(filtered_alpha[i])

            if y <0:
                right_xy.append([x,y])

            else:
                left_xy.append([x,y])



        filtered_r_numpy = np.array(filtered_r)
        filtered_alpha_numpy = np.array(filtered_alpha)
        # return np.transpose(np.array([np.cos(filtered_alpha_numpy)*filtered_r_numpy, np.sin(filtered_alpha_numpy)*filtered_r_numpy]))
        return right_xy,left_xy

    #old version of make marker
    def LiDAR_marker_old(self,points):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        # marker.type = marker.LINE_STRIP
        marker.type = marker.LINE_LIST
        marker.action = marker.MODIFY

        # #comment for qn1,2
        # marker.id = self.marker_id_number 
        # self.marker_id_number = self.marker_id_number +1

        # marker scale
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # marker orientaiton for qn 2 to compare with laser scan point cloud
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position for qn 2
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0


        
        # # marker orientaiton for qn 3,4 for mapping
        # marker.pose.orientation.x = self.q_x
        # marker.pose.orientation.y = self.q_y
        # marker.pose.orientation.z = self.q_z
        # marker.pose.orientation.w = self.q_w

        # # marker position for qn 3,4
        # marker.pose.position.x = self.x
        # marker.pose.position.y = self.y
        # marker.pose.position.z = 0.0
        # marker.lifetime = rospy.Duration()


        
        
        # marker.lifetime = rospy.time(10)

        # marker line points
        marker.points = []
        # add points to show in RVIZ
        for i in range(points.shape[0]):
            p = Point()
            p.x, p.y, p.z = points[i, 0], points[i, 1], 0
            marker.points.append(p)
        return marker

    
    def distance_of_point_from_line(self,test_point, line_start_point, line_end_point): 
        # print("test point is: ",test_point) 
        # print("start point of line is:",line_start_point)
        # print("End point of line is: ", line_end_point)
        # point to line distance, where the line is given with points line_start_point and line_end_point
        if np.all(np.equal(line_start_point, line_end_point)):
            #following line gives distance from test point to the defining point of line
            return np.linalg.norm(test_point-line_start_point)
        
        distance_of_point_from_line = np.divide(np.abs(np.linalg.norm(np.cross(line_end_point-line_start_point, line_start_point-test_point))), np.linalg.norm(line_end_point-line_start_point))
        
        
        return distance_of_point_from_line

    def get_farthest_point_fn(self,xy_points_raw):
        dmax = 0
        index = -1
        for i in range(1, len(xy_points_raw)):
            # xy_points_raw = np.array(xy_points_raw)
            # get distance of ith point from the start and end of xy_points_raw (the points with least and greatest angles)
            # we return the point with the maximum distance from the line connecting start and end of xy_points_raw
            distance_of_point = self.distance_of_point_from_line(xy_points_raw[i, :], xy_points_raw[0, :], xy_points_raw[-1, :])
            # print("distance_of_point is: ",distance_of_point)
            if (distance_of_point > dmax):
                index = i
                dmax = distance_of_point

        # print("farthest point is: ",xy_points_raw[index])
        return dmax, index

    ##################################################################################
    # The following version of split and merge does not work: because the entire vstack is printed
    # we need to do clustering and print individual clusters
    ##################################################################################


    # def SplitAndMerge(self,xy_points_raw, threshold):
    #     d, ind = self.GetMostDistant(xy_points_raw)

    #     if (d > threshold):
    #         #the following line splits from beginning till the index 
    #         #hence we get left array
    #         xy_points_raw_left_part = self.SplitAndMerge(xy_points_raw[:ind+1, :], threshold) 

    #         #the following line splits from the index till the end of the list
    #         #hence we get right array
    #         xy_points_raw_right_part = self.SplitAndMerge(xy_points_raw[ind:, :], threshold) 

    #         # there are 2 "d" points, so exlude 1 (for example from 1st array)
    #         #last point is removed from left_part
    #         points = np.vstack((xy_points_raw_left_part[:-1, :], xy_points_raw_right_part))
    #         print("points_after splitting into and stacking using vstack",points)
    #     else:
            
    #         #add first and last point to the vstack? why?
    #         #because these are the points across which when  line is made, dmax<threshold
    #         #but we need to send clusters of points

    #         points = np.vstack((xy_points_raw[0, :], xy_points_raw[-1, :]))
    #         print("points_after splitting into and stacking using vstack",points)
    #     return points



    def LiDAR_MARKER_2(self):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        # marker.type = marker.LINE_STRIP
        marker.type = marker.LINE_LIST
        marker.action = marker.MODIFY

        #comment for qn1,2
        marker.id = self.marker_id_number 
        self.marker_id_number = self.marker_id_number +1

        # marker scale
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # # marker orientaiton for qn 2 to compare with laser scan point cloud
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0

        # # marker position for qn 2
        # marker.pose.position.x = 0.0
        # marker.pose.position.y = 0.0
        # marker.pose.position.z = 0.0


        
        # marker orientaiton for qn 3,4 for mapping
        marker.pose.orientation.x = self.q_x
        marker.pose.orientation.y = self.q_y
        marker.pose.orientation.z = self.q_z
        marker.pose.orientation.w = self.q_w

        # marker position for qn 3,4
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.lifetime = rospy.Duration()

        # marker line points
        marker.points = []
        # add points to show in RVIZ
        
        return marker


    def temp_points_for_marker(self,mkr,pts):
        p1 = Point()
        p2 = Point()
        p1.x, p1.y, p1.z = pts[0][0], pts[0][1], 0
        p2.x, p2.y, p2.z = pts[-1][0], pts[-1][1], 0
        mkr.points.append(p1)
        mkr.points.append(p2)

    def distance_of_points(self,pt1,pt2):
        d = ((pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2)**0.5
        return d

        ##########
        #########
    def split_and_merge_algorithm_custom(self,xy_points_raw, threshold):
        xy_points_raw = np.array(xy_points_raw)

        point_distance, ind = self.get_farthest_point_fn(xy_points_raw)

        if (point_distance > threshold):
            #the following line splits from beginning till the index 
            #hence we get left array
            xy_points_raw_left_part = self.split_and_merge_algorithm_custom(xy_points_raw[:ind+1, :], threshold) 

            #the following line splits from the index till the end of the list
            #hence we get right array
            xy_points_raw_right_part = self.split_and_merge_algorithm_custom(xy_points_raw[ind:, :], threshold) 

        else:
            #mkr = self.MakeMarker2()
            flag = 0
            split_index = []

            #set max length of a line to ensure accuracy of map and higher resolution
            for i in range(len(xy_points_raw)):
                if i<len(xy_points_raw)-1 and self.distance_of_points(xy_points_raw[i],xy_points_raw[i+1])>0.05:
                    split_index.append(i)
                    flag = 1

            if flag == 1:
                prev = 0
                for i in range(len(split_index)):
                    #if i != split_index[-1]:
                    p1 = xy_points_raw[prev]
                    p2 = xy_points_raw[split_index[i]]
                    prev = split_index[i]+1

                    points_to_send = [p1, p2]
                    self.temp_points_for_marker(self.marker_2_new,points_to_send)
                
                p1 = xy_points_raw[prev]
                p2 = xy_points_raw[-1]
                points_to_send = [p1, p2]
                self.temp_points_for_marker(self.marker_2_new,points_to_send)
            
            else:
                self.temp_points_for_marker(self.marker_2_new,xy_points_raw)


    def callback_terrasentia_ekf(self, msg):
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

            self.orientation = np.array([self.q_x, self.q_y, self.q_z, self.q_w])

            self.theta = msg.pose.pose.position.y
            self.velocity_x = msg.twist.twist.linear.x
            self.yaw_rate = msg.twist.twist.angular.z



if __name__ == '__main__':
    rospy.init_node('main')
    #set threshold distance and id_number for markers to be unique
    Threshold_distance = 0.05
    id_number = 1

    pub_rviz = rospy.Publisher('visualization_msgs/MarkerArray', Marker, queue_size=0)
    ne = listenerNode()
    ne.run()

