#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import rosbag
import tf2_ros
import tf2_geometry_msgs
import math
import time
import numpy as np
import cv2

map_w = -1
map_h = -1
map_scale = -1
map_origin_x = -1
map_origin_y = -1
gt_x = -1
gt_y = -1
d = 500
img = np.ones((d,d,1),np.uint8)*33

def Maplistener():

    rospy.init_node('evalu_navi', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, readMap_cb)

    rospy.spin()

def mapIndex(mymap, i , j):
    return mymap.data[i + j * mymap.info.width]

def readMap_cb(data):
    global map_w
    global map_h
    global map_scale
    global map_origin_x
    global map_origin_y
    global gt_x
    global gt_y
    map_w = data.info.width
    map_h = data.info.height
    map_scale = data.info.resolution
    map_origin_x = data.info.origin.position.x + (map_w / 2) * map_scale
    map_origin_y = data.info.origin.position.y + (map_h / 2) * map_scale
    
    # read data in a specific location and measure the obstacles distance
    px = 1970 # is the same
    
    py = 4000 - 2239  # width 4000 - 1960 = 2040 like in gimp
    
    print "Got the map w and h", map_w, map_h
    print "This is the value of the cell at ", px, py
    print mapIndex(data,px,py) 
    
    min_dist = d
    idy = 0 
    for iy in range(py-d/2,py+d/2):
        idx = 0
        for ix in range(px-d/2,px+d/2):
            #print mapIndex(data,ix,iy)
            if (mapIndex(data,ix,iy) >= -2 and mapIndex(data,ix,iy) < 100 ):
                img[idx][idy] = 255
                #print ix, iy , "w" , idx , idy
            elif mapIndex(data,ix,iy) >= 100:
                img[idx][idy] = 0
                dist = abs((px-ix)) + abs((py-iy))
                if dist <= min_dist:
                    min_dist = dist
                    #print "i ve found one smaller dis" , min_dist            
                
                #print ix, iy , "b"  , idx , idy, mapIndex(data,ix,iy)
            else:
                print "mmm"
            idx = idx + 1
        idy = idy + 1
    print "mnin dist is ",min_dist
    cv2.circle(img,(int(idx/2),int(idy/2)), 5, (70,190,100), -1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()        
    vrpn_husky = topics=['/vrpn_client_node/husky/pose', 'geometry_msgs/PoseStamped']

if __name__ == '__main__':
    Maplistener()
    # Calculate the total distance traveled
    traveled_dist = 0
    init = True
    #bag = rosbag.Bag('/home/edo/Documents/husky_nav/0115/2019-01-15-16-33-14.bag')
    print "=== Calculating evaluation parameters ===" 
    #bag.close()
  

    '''
    ### Traveled distance  ###  topics=['/vrpn_client_node/husky/pose', 'geometry_msgs/PoseStamped' , '/tf_static', 'tf2_msgs/TFMessage']
    for topic, msg1, t in bag.read_messages():

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        trans = tfBuffer.lookup_transform('map', 'world',rospy.Time(0),rospy.Duration(1.0))
        wp_transformed = tf2_geometry_msgs.do_transform_pose(wp, trans)
    '''    
        

    
    



