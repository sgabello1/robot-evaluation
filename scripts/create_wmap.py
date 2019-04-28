#!/usr/bin/env python  
## this is for laoding theoric way points from txt file and convert to image map coordinates

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

rospy.init_node('pose_tf')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
trans = tfBuffer.lookup_transform('map', 'world',rospy.Time(0),rospy.Duration(6.0))
print sys.argv[1]
text_file = open(sys.argv[1], "r")
lines = text_file.readlines()
wp = PoseWithCovariance()
wpa = PoseWithCovariance()
wpb = PoseWithCovariance()
wpc = PoseWithCovariance()
wpd = PoseWithCovariance()

wp_transformed_st = PoseWithCovarianceStamped()
wpa_st = PoseWithCovarianceStamped()
wpb_st = PoseWithCovarianceStamped()
wpc_st = PoseWithCovarianceStamped()
wpd_st = PoseWithCovarianceStamped()

d = 4000
bw_color = 255
img = np.ones((d,d,1),np.uint8)*bw_color

# from crop_image script
dx = rospy.get_param("/create_wmap/window/x")
dy = rospy.get_param("/create_wmap/window/y")
cx = rospy.get_param("/create_wmap/changed_origin/x")
cy = rospy.get_param("/create_wmap/changed_origin/y")
bw_color = 255
img = np.ones((dy,dx,1),np.uint8)*bw_color
wb = 0.51
hb = 0.51
dc = 0.13
ds = 0.24

def readMap_cb(data):
    objectmap = data
    map_w = data.info.width
    map_h = data.info.height
    map_scale = data.info.resolution
    map_origin_x = data.info.origin.position.x 
    map_origin_y = data.info.origin.position.y 

    dcyl = dc/map_scale
    dstl = ds/map_scale

    cv2.destroyAllWindows()
    for i in range(0,len(lines)):
    	el = lines[i].split(',')
    	wp.pose.position.x = float(el[1])
    	wp.pose.position.y = float(el[2])
    	wp.pose.position.z = float(el[3])

    	wp.pose.orientation.x = float(el[4])
    	wp.pose.orientation.y = float(el[5])
    	wp.pose.orientation.z = float(el[6])
    	wp.pose.orientation.w = float(el[7].split('\n')[0])

    	wp_transformed = tf2_geometry_msgs.do_transform_pose(wp, trans)
    	wp_transformed_st.pose.pose.position = wp_transformed.pose.position
    	wp_transformed_st.pose.pose.orientation = wp_transformed.pose.orientation

    	point_px_m = wp_transformed_st.pose.pose.position.x / map_scale  # robot pose in map image coordinates
    	point_py_m = wp_transformed_st.pose.pose.position.y / map_scale 

    	point_px_mc = point_px_m + dx/2 - cx # robot pose in map cropped image coordinates
    	point_py_mc = point_py_m + dy/2 - cy

    	#time.sleep(2)
    	#print "WP ", i , "\n " ,wp_transformed_st
    	print "POINT ", el[0] ,"robot_px ", point_px_mc , "crop robot_py ", point_py_mc 
         
        if el[0] == "box":

            wpa.pose.position.x = wp.pose.position.x - wb/2
            wpa.pose.position.y = wp.pose.position.y - hb/2

            wpb.pose.position.x = wp.pose.position.x + wb/2
            wpb.pose.position.y = wp.pose.position.y - hb/2

            wpc.pose.position.x = wp.pose.position.x + wb/2
            wpc.pose.position.y = wp.pose.position.y + hb/2

            wpd.pose.position.x = wp.pose.position.x - wb/2
            wpd.pose.position.y = wp.pose.position.y + hb/2

            wpa_transformed = tf2_geometry_msgs.do_transform_pose(wpa, trans)
            wpb_transformed = tf2_geometry_msgs.do_transform_pose(wpb, trans)
            wpc_transformed = tf2_geometry_msgs.do_transform_pose(wpc, trans)
            wpd_transformed = tf2_geometry_msgs.do_transform_pose(wpd, trans)

            wpa_st.pose.pose.position = wpa_transformed.pose.position
            wpa_st.pose.pose.orientation = wpa_transformed.pose.orientation
            wpb_st.pose.pose.position = wpb_transformed.pose.position
            wpb_st.pose.pose.orientation = wpb_transformed.pose.orientation
            wpc_st.pose.pose.position = wpc_transformed.pose.position
            wpc_st.pose.pose.orientation = wpc_transformed.pose.orientation
            wpd_st.pose.pose.position = wpd_transformed.pose.position
            wpd_st.pose.pose.orientation = wpd_transformed.pose.orientation

            pax_m = int(wpa_st.pose.pose.position.x / map_scale + dx/2 - cx)
            pay_m = int(wpa_st.pose.pose.position.y / map_scale + dy/2 - cy)

            pbx_m = int(wpb_st.pose.pose.position.x / map_scale + dx/2 - cx)
            pby_m = int(wpb_st.pose.pose.position.y / map_scale + dy/2 - cy) 

            pcx_m = int(wpc_st.pose.pose.position.x / map_scale + dx/2 - cx)
            pcy_m = int(wpc_st.pose.pose.position.y / map_scale + dy/2 - cy)

            pdx_m = int(wpd_st.pose.pose.position.x / map_scale + dx/2 - cx)
            pdy_m = int(wpd_st.pose.pose.position.y / map_scale + dy/2 - cy)

            cv2.line(img,(pax_m,pay_m),(pbx_m,pby_m),(0,0,0),1)
            cv2.line(img,(pbx_m,pby_m),(pcx_m,pcy_m),(0,0,0),1)
            cv2.line(img,(pcx_m,pcy_m),(pdx_m,pdy_m),(0,0,0),1)
            cv2.line(img,(pdx_m,pdy_m),(pax_m,pay_m),(0,0,0),1)

        if el[0] == "cylinder":
            cv2.circle(img,(int(point_px_mc),int(point_py_mc)), int(dcyl), (0,0,0), -1)
        if el[0] == "stool":
            cv2.circle(img,(int(point_px_mc),int(point_py_mc)), int(dstl), (0,0,0), -1)
        
    cv2.imwrite('synth_map.png',img)

    cv2.imshow('image',img)
    cv2.waitKey(0)

def Maplistener():

    rospy.Subscriber("map", OccupancyGrid, readMap_cb)

    rospy.spin()

if __name__ == '__main__':
    Maplistener()