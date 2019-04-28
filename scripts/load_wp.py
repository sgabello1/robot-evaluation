#!/usr/bin/env python  
## this is for laoding theoric way points from txt file and convert to image map coordinates
## rosrun evalu_navi load_wp.py waypoints.txt
## rosbag play nice-map_2019-04-18-16-51-33.bag
import roslib 
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
import math
import time
import sys
import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2

rospy.init_node('load_wp')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
trans = tfBuffer.lookup_transform('map', 'world',rospy.Time(0),rospy.Duration(1.0))

map_file = rospy.get_param("/load_wp/map_path")
wp_file = rospy.get_param("/load_wp/path_to_save_wp")
text_file = open(map_file, "r")
lines = text_file.readlines()
wp = PoseWithCovariance()
wp_transformed_st = PoseWithCovarianceStamped()
d = 4000
bw_color = 33
img = np.ones((d,d,1),np.uint8)*bw_color

dx = rospy.get_param("/load_wp/window/x")
dy = rospy.get_param("/load_wp/window/y")
cx = rospy.get_param("/load_wp/changed_origin/x")
cy = rospy.get_param("/load_wp/changed_origin/y")


def readMap_cb(data):
    objectmap = data
    map_w = data.info.width
    map_h = data.info.height
    map_scale = data.info.resolution
    map_origin_x = data.info.origin.position.x 
    map_origin_y = data.info.origin.position.y 
    for i in range(0,len(lines)):
    	el = lines[i].split(',')
    	wp.pose.position.x = float(el[0])
    	wp.pose.position.y = float(el[1])
    	wp.pose.position.z = float(el[2])

    	wp.pose.orientation.x = float(el[3])
    	wp.pose.orientation.y = float(el[4])
    	wp.pose.orientation.z = float(el[5])
    	wp.pose.orientation.w = float(el[6].split('\n')[0])

    	wp_transformed = tf2_geometry_msgs.do_transform_pose(wp, trans)
    	wp_transformed_st.pose.pose.position = wp_transformed.pose.position
    	wp_transformed_st.pose.pose.orientation = wp_transformed.pose.orientation

    	robot_px_m = wp_transformed_st.pose.pose.position.x / map_scale  # robot pose in map image coordinates
    	robot_py_m = wp_transformed_st.pose.pose.position.y / map_scale 

    	robot_px_mc = robot_px_m + dx/2 - cx # robot pose in map cropped image coordinates
    	robot_py_mc = robot_py_m + dy/2 - cy

    	print "- WP - ", i ,"crop robot_px ", robot_px_mc , "crop robot_py ", robot_py_mc 
    	f =  open(wp_file, 'a+')
        f.write(str(str(robot_px_mc) + "," + str(robot_py_mc) + "\n"))
    f.close()

def Maplistener():

    rospy.Subscriber("map", OccupancyGrid, readMap_cb)

    rospy.spin()

if __name__ == '__main__':
    Maplistener()