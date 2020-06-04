#!/usr/bin/env python
import rospy

from joint_states import JointStates
from laser_subscriber import LaserSub
from laser_sub_fake import LaserSubFake
from odometry_subscriber import OdomSub
from ekf_sensor_pose_prediction import SensorPosePrediction

import copy
import time

from plot_visualizer import PlotVisual
from euclidean_minimum_spanning_tree import EMST
from clustering import Cluster
from coarse_level_association import CoarseLevel


class Track2DLidar():
    """ Tracking objects using 2D lidar.
    """

    #pub_laser = rospy.Publisher('/test/scan', LaserScan, queue_size=1)

    def __init__(self, process_odom_rate=10, process_laser_rate=3, wheelbase=None):
        # check settings
        while process_odom_rate < process_laser_rate:
            rospy.loginfo('[track_2D_lidar] process_odom_rate should not be smaller than process_laser_rate. Please terminate.')

        # joint states
        self.x = JointStates()

        # laser input
        self.laser = LaserSub()  # ros subscriber will loop again and again to update laser scan
        #self.laser = LaserSubFake()  # only one scan, will not update 

        # odom input
        self.odom = OdomSub()
        self.process_odom = SensorPosePrediction(wheelbase)

        # proces
        self.initialize()
        self.process_odom.initialize()
        self.run(process_odom_rate, process_laser_rate)

    def initialize(self):
        # prepare to run
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():  # wait until first scan is received so that laser is initialized
            if self.laser.first_recieved_done and self.odom.first_recieved_done:  # laser specs initialized by ROS sensor_msgs/LaserScan message
                break
            if not self.laser.first_recieved_done:
                rospy.loginfo('[track_2d_lidar] Waiting for laser input.')
            if not self.odom.first_recieved_done:
                rospy.loginfo('[track_2d_lidar] Waiting for odom and cmd_vel input.')
                
            r.sleep()

    def run(self, process_odom_rate, process_laser_rate):
        """ The working loop.
        """
        ########################
        # prepare for the loop
        ########################
        # scan visualize
        #plt = PlotVisual(axis=[-5.0,5.0,-5.0,5.0])
        
        # previous state
        laser_prev = None
        clusters_prev = None

        ########################
        # beginning of the loop with controled frequency
        ########################
        count = 0
        multiple = int(process_odom_rate/process_laser_rate)
        r = rospy.Rate(process_odom_rate)
        while not rospy.is_shutdown():
            count += 1
            # process lidar
            if count == multiple:
                count = 0
                _time = time.time()
                
                # --- 1. laser input (copy self.laser at this moment)
                laser_now = copy.deepcopy(self.laser)

                # --- 2. EMST (create input graph for segmentation) 
                mst = EMST(laser_now, neighbor_size=10)

                # --- 3. EGBIS (apply image segmentation technique to clustering)
                clusters = Cluster(mst, k=1.0, min_size=10)

                # --- 4. Coarse-level data association (ICP): update JointState.matched_track_indices
                CoarseLevel(laser_now, laser_prev, clusters, clusters_prev, self.x, tentative_threshold = 5, icp_max_dist=0.5)

                # --- 5. Fine-level data association
                

                # --- visualization 
                _laser_plot = True
                #plt.visualize(laser_now, [_laser_plot, mst, clusters])
                #plt.visualize(laser_now, [_laser_plot, None, None])
                #plt.visualize(laser_now, [None, mst, None])
                #plt.visualize(laser_now, [None, None, clusters])

                # --- end of the loop
                laser_prev = laser_now
                clusters_prev = clusters

                # --- check loop rate
                duration = time.time() - _time
                rospy.loginfo(duration)
                if duration > 1.0/process_laser_rate:
                    rospy.loginfo('[track_2d_lidar] Processing laser missed its control loop. It actually takes %f secs.' % duration)
     
            # process odom
            else:
                _time = time.time()
                # process odom
                self.process_odom.run(odom=self.odom, x=self.x)

                #  check loop rate
                duration = time.time() - _time
                #rospy.loginfo(duration)
                if duration > 1.0/process_odom_rate:
                    rospy.loginfo('[track_2d_lidar] Processing odom missed its control loop. It actually takes %f secs.' % duration)
            
            r.sleep()

        ########################
        # clean up after the loop
        ########################
        #plt.show()
    