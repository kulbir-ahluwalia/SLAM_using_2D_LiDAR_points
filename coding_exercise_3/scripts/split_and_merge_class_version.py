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

        self.velocity_x = 0.0
        # self.vy = 0.0
        self.yaw_rate = 0.0

        self.orientation = np.array([self.q_x, self.q_y, self.q_z, self.q_w])

        # Following are for laser scan
        self.angle_min = -2.356194496154785
        self.angle_max = 2.356194496154785
        self.angle_increment = 0.004363323096185923

        self.angles_of_lidar = np.linspace(
            self.angle_min, self.angle_max, 1081)
        print("angles from lidar are: ", self.angles_of_lidar)
        self.range_values = np.array([])

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

            self.linearvelocity()  # call the function used to create the Odometry ROS message
            # self.linearvelocity_gps()

            # print("listenerNode.ekf_x is: ",listenerNode.ekf_x)
            # print("listenerNode.ekf_y is: ",listenerNode.ekf_y)

            # this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
            self.br.sendTransform(
                # (self.x, self.y, 0.0), self.orientation, rospy.Time.now(), "/base_link", "/map")
                (self.x, self.y, 0.0), self.orientation, rospy.Time.now(), "/camera_link", "/map")

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

    def callback_laser_scan(self, msg):
        # self.angle_min = msg.angle_min
        # self.angle_max = msg.angle_max
        # self.angle_increment = msg.angle_increment

        # print("angle details are: ",self.angle_increment,self.angle_max,self.angle_min)

        self.range_values = np.array(msg.ranges)
        print("type of range_values", type(self.range_values))
        print("length of range_values", len(self.range_values))
        print("range_values numpy array is:", self.range_values)

        # since angle increment and min and max angle does not change, we set the values

        self.x_value_lidar = np.cos(self.angles_of_lidar)*self.range_values
        self.y_value_lidar = np.sin(self.angles_of_lidar)*self.range_values

        print("x values from lidar are:", self.x_value_lidar)
        print("y values from lidar are:", self.y_value_lidar)

        # plt.plot(self.y_value_lidar, self.x_value_lidar)
        # plt.grid(True)

        global pub_rviz
        global threshold
        global isPrint
        ranges = np.asarray(msg.ranges)
        # alpha = np.linspace(data.angle_min,data.angle_max,360)
        alpha = np.linspace(msg.angle_min, msg.angle_max, 1081)

        # ranges[ranges == np.inf] = 2  # clip the distance to 3m

        # ranges[ranges <= 0.019999] = 0
        # ranges[ranges > 30] = 0

        # ranges = ranges[ (ranges >= 0.019999) & (ranges <= 30) ]

        xy_points_raw = self.Polar2Cartesian(ranges, alpha)
        print("xy_points_raw shape is",xy_points_raw.shape)
        print("xy_points_raw are: ",xy_points_raw)


        points = self.SplitAndMerge(xy_points_raw, threshold)
        print("points are: ",points)
        # for point in points:
        #     plt.scatter(point[0],point[1])
        # plt.show()

        for i in range(points.shape[0]-1):
            ro, alpha = self.GetPolar(points[i:i+2, 0], points[i:i+2, 1])
            ro, alpha = self.CheckPolar(ro, alpha)
            if isPrint:
                print("ro = {}, alpha = {}".format(ro, alpha))
        if isPrint:
            print("-------------------------------------")
        # make marker for RVIZ and publish it
        marker = self.MakeMarker(points)
        pub_rviz.publish(marker)
        # plt.show()


    def Polar2Cartesian(self,r, alpha):
        filtered_r = []
        filtered_alpha = []
        for i in range(0,len(r)):
            # if r[i]>=0.01999 and r[i]<30:
            if r[i]>=0.02 and r[i]<25:
                print("r is: ",r[i])
                filtered_r.append(r[i])
                filtered_alpha.append(alpha[i])

        filtered_r_numpy = np.array(filtered_r)
        filtered_alpha_numpy = np.array(filtered_alpha)
        print("filtered_r is:",filtered_r)
        print("shape of filtered r is:", filtered_r_numpy.shape)

        print("filtered_alpha is:",filtered_alpha)
        print("shape of filtered alpha is:", filtered_alpha_numpy.shape)
        return np.transpose(np.array([np.cos(filtered_alpha_numpy)*filtered_r_numpy, np.sin(filtered_alpha_numpy)*filtered_r_numpy]))


    def Cartesian2Polar(self,x, y):
        r = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return r, phi


    def MakeMarker(self,points):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.LINE_STRIP
        marker.action = marker.MODIFY

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
        # marker orientaiton
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0

        marker.pose.orientation.x = self.q_x
        marker.pose.orientation.y = self.q_y
        marker.pose.orientation.z = self.q_z
        marker.pose.orientation.w = self.q_w

        # marker position
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
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


    def GetPolar(self,X, Y):
        # center the data
        X = X-np.mean(X)
        Y = Y-np.mean(Y)
        # fit line through the first and last point (X and Y contains 2 points, start and end of the line)

        #polyfit returns m=slope and intercept c, in that order. 
        k, n = np.polyfit(X, Y, 1)
        print("k and n are:",k,n)
        alpha = m.atan(-1/k)  # in radians
        ro = n/(m.sin(alpha)-k*m.cos(alpha))
        return ro, alpha


    def CheckPolar(self,ro, alpha):
        if ro < 0:
            alpha = alpha + m.pi
            if alpha > m.pi:
                alpha = alpha-2*m.pi
            ro = -ro
        return ro, alpha


    def getDistance(self,xy_points_raw, Ps, Pe):  
        # point to line distance, where the line is given with points Ps and Pe
        if np.all(np.equal(Ps, Pe)):
            return np.linalg.norm(xy_points_raw-Ps)
        return np.divide(np.abs(np.linalg.norm(np.cross(Pe-Ps, Ps-xy_points_raw))), np.linalg.norm(Pe-Ps))


    def GetMostDistant(self,xy_points_raw):
        dmax = 0
        index = -1
        for i in range(1, xy_points_raw.shape[0]):
            d = self.getDistance(xy_points_raw[i, :], xy_points_raw[0, :], xy_points_raw[-1, :])
            if (d > dmax):
                index = i
                dmax = d
        return dmax, index


    def SplitAndMerge(self,xy_points_raw, threshold):
        d, ind = self.GetMostDistant(xy_points_raw)

        if (d > threshold):
            #the following line splits at the index 
            xy_points_raw_left_part = self.SplitAndMerge(xy_points_raw[:ind+1, :], threshold)  # split and merge left array
            xy_points_raw_right_part = self.SplitAndMerge(xy_points_raw[ind:, :], threshold)  # split and merge right array
            # there are 2 "d" points, so exlude 1 (for example from 1st array)
            points = np.vstack((xy_points_raw_left_part[:-1, :], xy_points_raw_right_part))
        else:
            points = np.vstack((xy_points_raw[0, :], xy_points_raw[-1, :]))
        return points

    # def SplitAndMerge(self,xy_points_raw, threshold):
    #     d, ind = self.GetMostDistant(xy_points_raw)






    # def callback(self,data):
    #     global pub_rviz
    #     global threshold
    #     global isPrint
    #     ranges = np.asarray(data.ranges)
    #     # alpha = np.linspace(data.angle_min,data.angle_max,360)
    #     alpha = np.linspace(data.angle_min, data.angle_max, 1081)

    #     ranges[ranges == np.inf] = 30  # clip the distance to 30m
    #     P = self.Polar2Cartesian(ranges, alpha)
    #     points = self.SplitAndMerge(P, threshold)
    #     for i in range(points.shape[0]-1):
    #         ro, alpha = self.GetPolar(points[i:i+2, 0], points[i:i+2, 1])
    #         ro, alpha = self.CheckPolar(ro, alpha)
    #         if isPrint:
    #             print("ro = {}, alpha = {}".format(ro, alpha))
    #     if isPrint:
    #         print("-------------------------------------")
    #     # make marker for RVIZ and publish it
    #     marker = self.MakeMarker(points)
    #     pub_rviz.publish(marker)


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


    # def manual(self,pub, in1, in2):
    #     global vel
    #     vel.linear.x, vel.linear.y, vel.linear.z = in1, 0, 0
    #     vel.angular.x, vel.angular.y, vel.angular.z = 0, 0, in2
    #     pub.publish(vel)
    #     return


# # Main function.
# if __name__ == '__main__':
#     # Initialize the node and name it.

#     rospy.init_node('LiDAR_example_node/MarkerArray', anonymous = True)

#     # pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 3)


#     # Go to the main loop.
#     ne = listenerNode()
#     ne.run()


if __name__ == '__main__':
    rospy.init_node('main')
    threshold = 0.05
    id_number = 1
    vel = Twist()

    pub_rviz = rospy.Publisher('visualization_msgs/MarkerArray', Marker, queue_size=0)
    # pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=3)

    # rospy.Subscriber("scan", LaserScan, callback)
    # /terrasentia/scan
    

    isPrint = False
    ne = listenerNode()
    ne.run()

	# while True:
	# 	print("\nProvide robot velocities ('e' to exit,'p' to start printing lines): ")


	# 	inp = raw_input()


	# 	if inp == 'e':
	# 		break
	# 	if inp == 'p':
	# 		isPrint = True
	# 		continue
	# 	else:
	# 		isPrint = False
	# 		try:
	# 			in1,in2 = inp.split(' ') print("in1 and in2 are: ",in1,in2)
	# 			in1 = float(in1)
	# 			in2 = float(in2)
	# 		except:
	# 			print("##### WRONG INPUTS #####")
	# 			continue
	# 		manual(pub_vel,in1,in2)
