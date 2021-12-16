# SLAM_using_2D_LiDAR_points

Please refer the report for this exercise for detailed explanation and images of the resulting map from 2D point clouds. 

Install the following package before you begin:
```
sudo apt-get install python-tk
```
Commands to run the code and the rosbag (run each command in a separate terminal in the order below): 
```
roscore
rosbag play -l 2020-11-13-14-39-36.bag
python split_and_merge_class_version.py
rviz #(then add marker in the list of topics to visualize)
```
