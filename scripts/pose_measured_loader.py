#!/usr/bin/env python  
## this is for laoding from rosbag file and convert all the robot poses to image map coordinates
## rosrun evalu_navi pose_measured_loader.py nice-map_2019-04-18-16-51-33.bag
import roslib 
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, TransformStamped
import math
import time
import sys
import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2
import rosbag
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from move_base_msgs.msg import MoveBaseActionResult
from tf import transformations as tff

rospy.init_node('pose_measure')
wp = PoseWithCovariance()
wp_transformed_st = PoseWithCovarianceStamped()
d = 4000
bw_color = 33
img = np.ones((d,d,1),np.uint8)*bw_color

dx = rospy.get_param("/pose_measured_loader/window/x")
dy = rospy.get_param("/pose_measured_loader/window/y")
cx = rospy.get_param("/pose_measured_loader/changed_origin/x")
cy = rospy.get_param("/pose_measured_loader/changed_origin/y")
robot = rospy.get_param("/pose_measured_loader/robot")
wp_dump_file = rospy.get_param("/pose_measured_loader/wp_dump_file")

def readMap_cb(data):
    objectmap = data
    map_w = data.info.width
    map_h = data.info.height
    map_scale = data.info.resolution
    map_origin_x = data.info.origin.position.x 
    map_origin_y = data.info.origin.position.y 
    
    bag = rosbag.Bag(sys.argv[1])
    bag2 = rosbag.Bag(sys.argv[2])

    tf_static  = bag.read_messages(topics=['/tf_static', 'tf2_msgs/TFMessage'])
    vrpn_robot = bag2.read_messages(topics=['/vrpn_client_node/'+robot+'/pose', 'geometry_msgs/PoseStamped'])
    moveb_result = bag.read_messages(topics=['/move_base/result', 'move_base_msgs/MoveBaseActionResult'])
    prev_tt = rospy.Time(0) # previous time stamp
    thr_dt = rospy.Duration(10) # threshold between each move base result timestamp in seconds
    tt_vector = np.array([])
    
    for topic, msg, t in tf_static:
        # we invert the matrix to get world to map
        rot_w2m = msg.transforms[0].transform.rotation 
        tras_w2m = msg.transforms[0].transform.translation

    m2w = tff.concatenate_matrices(tff.translation_matrix([tras_w2m.x,tras_w2m.y,tras_w2m.z]), tff.quaternion_matrix([rot_w2m.x,rot_w2m.y,rot_w2m.z,rot_w2m.w]))
    w2m = tff.inverse_matrix(m2w)
    
    world2map = TransformStamped()
    world2map.transform.rotation = rot_w2m
    world2map.transform.rotation.w = -1 * rot_w2m.w # inverse rotation matrix just using quaternions
    world2map.transform.translation.x = w2m[0][3]
    world2map.transform.translation.y = w2m[1][3]
    world2map.transform.translation.z = w2m[2][3]

    ii = 1 # index for saved time stamp into the vector tt_vector 
    thr_dt = rospy.Duration(0.001) # threshold between time stamp in the saved from move based and poses
    for topic, msg, t in vrpn_robot:
	    robot_w = msg
	    robot_m = tf2_geometry_msgs.do_transform_pose(robot_w, world2map) #transform robot pose from world ref to map ref 

	    robot_px_m = robot_m.pose.position.x / map_scale  # robot pose in map image coordinates
	    robot_py_m = robot_m.pose.position.y / map_scale 

	    robot_px_mc = robot_px_m + dx/2 - cx # robot pose in map cropped image coordinates
	    robot_py_mc = robot_py_m + dy/2 - cy

	    #print  robot_px_mc , robot_py_mc , msg.header.stamp # all timestamps
	    f =  open(wp_dump_file+ str(ii-1)+ str(ii) + robot + '.txt', 'a+')
	    f.write(str(str(robot_px_mc) + "," + str(robot_py_mc) + "," + str(msg.header.stamp) + "\n"))

    f.close()
    bag.close()
    sub_once.unregister()
    rospy.signal_shutdown("reason")


def Maplistener():
    global sub_once
    sub_once = rospy.Subscriber("map", OccupancyGrid, readMap_cb) 

    rospy.spin()

if __name__ == '__main__':
    Maplistener()