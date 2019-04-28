#!/usr/bin/env python
# calculate distance, curvature and bending energy from image map coordinates

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
map_w = -1
map_h = -1
map_scale = -1
map_origin_x = -1
map_origin_y = -1
gt_x = -1
gt_y = -1
objectmap = OccupancyGrid()
d = 50 # window in which distance from obstacles is calculated
bw_color = 33
img = np.ones((d,d,1),np.uint8)*bw_color
obs_dist_path = np.array([0,0])
all_k = np.array([0,0])
sample_time = 0.1 # seconds

# to compute image map coordinates
dx = 300
dy = 300
cx = 30    #changing the orgining for cropping
cy = -65

t_start = 0#1547541219.645047921
t_stop =  1554795482.794750496

def mapIndex(mymap, i , j):
    return mymap.data[i + j * mymap.info.width]

def calcMinDistFromObstacles(data,px,py):
    # read data in a specific location and measure the obstacles distance
    # calcuate minimum distance from obstacles
    obs_dist = d
    idy = 0 
    for iy in range(py-d/2,py+d/2):
        idx = 0
        for ix in range(px-d/2,px+d/2):
            #print mapIndex(data,ix,iy)
            if (mapIndex(data,ix,iy) >= -1 and mapIndex(data,ix,iy) < 100 ):
                img[idx][idy] = 255
                #print ix, iy , "w" , idx , idy
            elif mapIndex(data,ix,iy) >= 100:
                img[idx][idy] = 0
                dist = math.sqrt((px-ix)**2 + (py-iy)**2)
                if dist <= obs_dist:
                    obs_dist = dist
            else:
                print "mmm something went bad"
            idx = idx + 1
        idy = idy + 1
    '''
    cv2.circle(img,(int(idx/2),int(idy/2)), 5, (70,190,100), -1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    '''
    return obs_dist


def PathCalculations(bag):
    # here we get the tranformation between map and world reference and robot poses according to world coordinates
    tf_static  = bag.read_messages(topics=['/tf_static', 'tf2_msgs/TFMessage'])
    vrpn_robot = bag.read_messages(topics=['/vrpn_client_node/husky/pose', 'geometry_msgs/PoseStamped'])

    
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
    
    t_old = rospy.Time(0)
    init = True
    dist = 0
    deltax = 0
    dfx_old = 0
    dfx = 0
    deltay = 0
    dfy_old = 0
    dfy = 0
    bte = 0

    global obs_dist_path
    global all_k
    print "start processing like crazy"
    
    for topic, msg, t in vrpn_robot:
        #sys.stdout.flush()
        robot_w = msg
        robot_m = tf2_geometry_msgs.do_transform_pose(robot_w, world2map) #transform robot pose from world ref to map ref 
        robot_px_m = robot_m.pose.position.x / map_scale # robot pose in map image coordinates
        robot_py_m = robot_m.pose.position.y / map_scale
 
        robot_px_m_dist = int(robot_px_m + map_w/2)
        robot_py_m_dist = int(robot_py_m + map_h/2)

        robot_px_mc = (robot_px_m + dx/2 - cx)#*map_scale # robot pose in map cropped image coordinates [meters]
        robot_py_mc = (robot_py_m + dy/2 - cy)#*map_scale
        
        
        dt = abs((t.secs + t.nsecs/10**9) - (t_old.secs + t_old.nsecs/10**9))
 
        if t > rospy.Time(t_stop):
            break

        if t < rospy.Time(t_start):
            pass
        else:
            # calculating total traveled distance and curvature
            if init:
                init = False
            else:
                #deltax = robot_m.pose.position.x - x_old
                deltax = robot_px_mc - x_old
                #deltay = robot_m.pose.position.y - y_old
                deltay = robot_py_mc - y_old
                dist = dist + math.sqrt(deltax**2 + deltay**2) # distance traveled

                if dt > 0.0:
                    dfx = deltax/dt # first derivate for the curvature
                    dfy = deltay/dt
                    ddfx = (dfx - dfx_old)/dt # second derivate
                    ddfy = (dfy - dfy_old)/dt
                    if abs(dfx) > 0.0 and abs(dfy) > 0.0:
                        k_num = abs(dfx*ddfy - dfy*ddfx) # wikipedia s formula for curvature
                        k_den = (dfx**2 + dfy**2)**(1.5)
                        k = k_num / k_den

                       # print k_num , k_den
                    else:
                        k = 0
                    all_k = np.vstack([all_k,[k,t]])
                    bte = bte + k**2
            print robot_px_mc, robot_py_mc, t
        x_old = robot_px_mc
        y_old = robot_py_mc
        dfx_old = dfx
        dfy_old = dfy
        t_old = t

        # calculating minimum distance from obstacles with a sampling time
        '''
        if dt >= sample_time:
            obs_dist = calcMinDistFromObstacles(objectmap,robot_px_m_dist,robot_py_m_dist)
            obs_dist_path = np.vstack([obs_dist_path,[obs_dist,t]])
            t_old = t
        '''

    #min_obs_dist = np.amin(obs_dist_path[1:,0])
    #min_obs_dist_index = np.where(obs_dist_path[1:,0] == np.amin(obs_dist_path[1:,0]))
    #index_min = min_obs_dist_index + np.ones(len(min_obs_dist_index) ,np.uint8)
    #time_obs_dist = obs_dist_path[index_min,1]
    #print "Absolute minimum distance ", min_obs_dist , " at " , time_obs_dist
    print "Total distance traveled ",dist
    print "Total bending energy ", bte
    max_k = np.amax([all_k[:,0]])
    print "Max curvature ", max_k

    with open('Estimation.txt', 'w') as file:
        file.write('1 - obs_dist_path\n2 - dist\n3 - all_k\n with a sample time of ')
        file.write(str(sample_time))
        file.write('\n\n\n')
        #file.write(str(obs_dist_path[:,:]))
        #file.write('\n \n \n')
        file.write(str(dist))
        file.write('\n \n \n')
        file.write(str(all_k[:,:]))
    
    print "All finished. Check ./Estimation.txt file for details" 
    
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
    map_origin_x = data.info.origin.position.x 
    map_origin_y = data.info.origin.position.y 

    bag = rosbag.Bag(sys.argv[1])
    PathCalculations(bag)
    bag.close() 

def Maplistener():

    rospy.init_node('evalu_navi', anonymous=True)

    rospy.Subscriber("map", OccupancyGrid, readMap_cb)

    rospy.spin()

if __name__ == '__main__':
    Maplistener()
  

    
    



