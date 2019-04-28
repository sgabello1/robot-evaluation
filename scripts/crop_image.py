#!/usr/bin/env python
## this is crop the image map and check if the waypoint are in the right place
## 

import rospy
from std_msgs.msg import String, Time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import rosbag
import tf2_ros
import tf2_geometry_msgs
import math
import time
import numpy as np
import cv2
import sys
from tf import transformations as tff
np.set_printoptions(threshold=sys.maxsize)

# init params
# "/media/edo/My\ Passport/husky\ nav\ test/Daniele/250419-husky-apollo-complex-map/map.txt"
map_w = -1
map_h = -1
map_scale = -1
map_origin_x = 2000 # i know that from experiments
map_origin_y = 2000
gt_x = -1
gt_y = -1
objectmap = OccupancyGrid()

dx = rospy.get_param("/crop_image/window/x")
dy = rospy.get_param("/crop_image/window/y")
cx = rospy.get_param("/crop_image/changed_origin/x")
cy = rospy.get_param("/crop_image/changed_origin/y")
bw_color = 33
img = np.ones((dy,dx,1),np.uint8)*bw_color
obs_dist_path = np.array([0,0])
all_k = np.array([0,0])

def mapIndex(mymap, i , j):
    return mymap.data[i + j * mymap.info.width]

def cropMap(data,px,py,dx,dy):
    # read data in a specific location and measure the obstacles distance
    # calcuate minimum distance from obstacles
    idy = 0 
    for iy in range(py-dy/2,py+dy/2):
        idx = 0
        for ix in range(px-dx/2,px+dx/2):
            if (mapIndex(data,ix,iy) >= -1 and mapIndex(data,ix,iy) < 100 ):
                img[idy][idx] = 255
            elif mapIndex(data,ix,iy) >= 100:
                img[idy][idx] = 0
            else:
                print "mmm something went bad"
            idx = idx + 1
        idy = idy + 1

    return img
    
def readMap_cb(data):
    global objectmap
    global map_w
    global map_h
    global map_scale
    global map_origin_x
    global map_origin_y
    global gt_x
    global gt_y
    objectmap = data
    map_w = data.info.width
    map_h = data.info.height
    map_scale = data.info.resolution

    #img = cropMap(data,2120,1900,dx,dy)
    ocx = map_origin_x + cx
    ocy = map_origin_y + cy

    img = cropMap(data,ocx,ocy,dx,dy) 
    
    cv2.imwrite('map_cropped.png',img)
      
    cv2.circle(img,(25,122), 5, (70,190,100), -1)
    cv2.circle(img,(11,22), 5, (153,190,100), -1)
    '''
    #cv2.circle(img,(180,106), 5, (70,190,100), -1)
    cv2.circle(img,(137,78), 5, (70,190,100), -1)
    cv2.circle(img,(109,152), 5, (70,190,100), -1)
    cv2.circle(img,(169,149), 5, (70,190,100), -1)
    '''
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def Maplistener():

    rospy.init_node('evalu_navi', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, readMap_cb)

    rospy.spin()

if __name__ == '__main__':
    Maplistener()
    
  

    
    



