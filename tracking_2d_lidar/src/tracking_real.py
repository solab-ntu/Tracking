#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import time
import copy
from icp import icp

import rospy
from laser_sub_fake import LaserSubFake
from euclidean_minimum_spanning_tree import EMST
from clustering import Cluster
from laser_odom_sub import LaserOdomSub
from joint_states import State
from coarse_level_association import CoarseLevel
from visualize_tracking import VisualizeTracking
from visualize_from_car import VisualizeFromCar
from process_kf_bagfile import ProcessKF

from get_bag_info import get_duration
import csv
from plot_visualize import plot_mst, scatter_laser, plot_segmentation  # for Thesis

def realtime():
    # params
    k = rospy.get_param('tracking/EGBIS/k', 1.0)   # node_name/a/a
    min_size = rospy.get_param('tracking/EGBIS/min_size', 5)
    icp_max_dist = rospy.get_param('tracking/ICP/icp_max_dist', 1.0)
    inflation_size = rospy.get_param('tracking/CheckStatic/inflation_size', 8)
    speed_threshold = rospy.get_param('tracking/CheckStatic/speed_threshold', 0.2)
    static_threshold = rospy.get_param('tracking/CheckStatic/static_threshold', 0.7)

    use_amcl = rospy.get_param('tracking/use_amcl', True)

    # rate
    rate = 5

    # joint states
    x = State(rate)

    # laser_odom sub
    laser_odom = LaserOdomSub(use_amcl)  

    # process kf
    kf = ProcessKF(x, inflate_size=inflation_size, static_threshold=static_threshold, speed_threshold=speed_threshold, prediction_time=3.0)

    # init loop
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():  # wait until first scan is received so that laser_odom is initialized
        if laser_odom.first_cartesian_done:  # laser_odom specs initialized by ROS sensor_msgs/LaserScan message
            break
        else:
            rospy.loginfo('[test_coarse_level] Waiting for laser_odom_sub to init.')
        r.sleep()

    laser_prev = None
    clusters_prev = None

    # init plot
    vt_plt = VisualizeTracking(x, kf)
    # vt_plt = VisualizeFromCar(laser_odom, x)
    
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        # ========================================= loop =========================================
        _time = time.time()

        # --- 1. laser_odom input (copy self.laser_odom at this moment)
        laser_odom.is_copying = True
        while(laser_odom.updated == False):   
            pass
        laser_odom_now = copy.deepcopy(laser_odom)
        laser_odom.is_copying = False

        assert len(laser_odom.cartesian) == len(laser_odom_now.cartesian)

        # scatter_laser(laser_odom_now)
        #time_1 = time.time()
        #duration_1 = time_1 - _time

        # --- 2. EMST (create input graph for segmentation) 
        mst = EMST(laser_odom_now, neighbor_size=30)

        # plot_mst(mst, laser_odom_now)
        #time_2 = time.time()
        #duration_2 = time_2 - time_1

        # --- 3. EGBIS (apply image segmentation technique to clustering)
        clusters_now = Cluster(mst, k=k, min_size=min_size)

        # plot_segmentation(clusters_now, laser_odom_now)
        # time_3 = time.time()
        # duration_3 = time_3 - time_2

        # --- 4. Coarse-level data association (ICP): update JointState.track_indices
        CoarseLevel(laser_odom_now, laser_prev, clusters_now, clusters_prev, x, tentative_threshold=0, icp_max_dist=icp_max_dist)
        # time_4 = time.time()
        # duration_4 = time_4 - time_3

        # --- 5. KF
        kf.predict_update(laser_odom_now, clusters_now, rate)
        #kf.pub_estimated_vel()
        kf.publish_to_costmap()
        # kf.publish_kf_state(laser_odom_now)
        kf.publish_static_scan()
        
        #time_5 = time.time()
        #duration_5 = time_5 - time_4
        
        # --- 6. visualization
        # vt_plt.plot(laser_odom_now)
        # vt_plt.plot()
        #time_6 = time.time()
        #duration_6 = time_6 - time_5

        # --- end of loop
        laser_prev = laser_odom_now
        clusters_prev = clusters_now

        # --- efficiency monitor
        duration = time.time() - _time
        # rospy.loginfo(duration_1)
        # rospy.loginfo(duration_2)
        # rospy.loginfo(duration_3)
        # rospy.loginfo(duration_4)
        # rospy.loginfo(duration_5)
        # rospy.loginfo(duration_6)
        # rospy.loginfo(duration)
        # rospy.loginfo('-------------')
        
        if duration > 1.0/rate:
            rospy.loginfo('[tracking] tracking missed its control loop. It actually takes %f secs.' % duration)
        r.sleep()

        # ========================================= loop =========================================



if __name__ == "__main__":
    rospy.init_node('tracking', anonymous=True, disable_signals=True)      
    realtime()
    #rospy.spin()
