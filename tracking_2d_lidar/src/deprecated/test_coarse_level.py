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
from laser_subscriber import LaserSub
from joint_states_slam import JointStates
from coarse_level_association_slam import CoarseLevel

def realtime_test():
    # rate
    rate = 3

    # --- joint states
    x = JointStates()

    # laser sub
    laser = LaserSub()

    # init loop
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():  # wait until first scan is received so that laser is initialized
        if laser.first_recieved_done:  # laser specs initialized by ROS sensor_msgs/LaserScan message
            break
        else:
            rospy.loginfo('[test_coarse_level] Waiting for laser to init.')
        r.sleep()

    laser_prev = None
    clusters_prev = None

    # init plot
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Clusters Tracking', fontsize=22)
    ax.axis([-5.0,5.0,-5.0,5.0])
    ax.plot([0], [0], marker='>', markersize=20, color="blue")  # plot the origin
    

    
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        # ========================================= loop =========================================
        _time = time.time()

        # --- 1. laser input (copy self.laser at this moment)
        laser_now = copy.deepcopy(laser)
        #time_1 = time.time()
        #duration_1 = time_1 - _time

        # --- 2. EMST (create input graph for segmentation) 
        mst = EMST(laser_now, neighbor_size=10)
        #time_2 = time.time()
        #duration_2 = time_2 - time_1

        # --- 3. EGBIS (apply image segmentation technique to clustering)
        clusters_now = Cluster(mst, k=3.0, min_size=5)
        #time_3 = time.time()
        #duration_3 = time_3 - time_2

        # --- 4. Coarse-level data association (ICP): update JointState.matched_track_indices
        CoarseLevel(laser_now, laser_prev, clusters_now, clusters_prev, x, tentative_threshold = 10, icp_max_dist=1.0)
        #time_4 = time.time()
        #duration_4 = time_4 - time_3

        # --- 5. fake fine-level (no measurement matching, just replace with new ones)
        assert len(x.xt) == len(x.xp)
        assert len(x.matched_track_indices) == len(clusters_now.clusters)

        for i in range(len(x.xt)):
            xp_i = np.array([[],[]])
            # collect all cluster that matches this track
            for cluster, track_i in zip(clusters_now.clusters, x.matched_track_indices):
                if track_i == i:
                    x_i = laser_now.cartesian[cluster,0]
                    y_i = laser_now.cartesian[cluster,1]
                    p_i = np.array([list(x_i), 
                                    list(y_i)])
                    xp_i = np.concatenate((xp_i, p_i), axis=1)
            # xp
            x.xp[i] = xp_i
            # xt
            gamma_i = np.mean(x.xp[i][0])
            delta_i = np.mean(x.xp[i][1])
            dx = gamma_i - x.xt[i][0]
            dy = delta_i - x.xt[i][1]
            xt_i = np.array([[gamma_i],  # x
                            [delta_i],  # y  
                            [0],        # theta
                            [dx],        # vx
                            [dy],        # vy
                            [0]])       # vtheta
            x.xt[i] = xt_i

        #time_5 = time.time()
        #duration_5 = time_5 - time_4

        # --- 6. visualization
        #colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']*20
        ax_list = []
        for xp_i, xt_i, xt_counter, xt_id in zip(x.xp, x.xt, x.xt_counter, x.xt_id):
            
            # xt - arrow
            _x = xt_i[0][0]
            _y = xt_i[1][0]
            dx = (xt_i[3][0])*5  # enlarge the arrow length
            dy = (xt_i[4][0])*5
            a = ax.arrow(_x, _y, dx, dy, width=0.0001, head_width=0.05)
            ax_list.append(a)
            
            # xt - counter 
            if xt_counter < 0:  
                color = 'g'  # mature
                text = str(xt_id)
            else:
                color = 'r'  # tentative
                text = str(xt_id) + '(' + str(xt_counter) + ')'

            # xp- box
            left = min(xp_i[0])
            bottom = min(xp_i[1])
            width = abs(max(xp_i[0])-left)
            height = abs(max(xp_i[1])-bottom)
            p = plt.Rectangle((left, bottom), width, height, fill=False, edgecolor=color, linewidth=2)
            #p.set_transform(ax.transAxes)
            #p.set_clip_on(False)
            a = ax.add_patch(p)
            ax_list.append(a)
            
            # xt - text
            top = bottom + height + 0.1
            a = ax.text(left, top, text, color=color, fontsize=16)
            ax_list.append(a)

            # xp - measurements
            a = ax.scatter(xp_i[0], xp_i[1], s=25.0, c='k', linewidths=0)
            ax_list.append(a)
        
        # plot
        plt.pause(1e-12)  # pause for real time display
        for a in ax_list:  # clear data on the plot
            a.remove()

        #time_6 = time.time()
        #duration_6 = time_6 - time_5

        # --- end of loop
        laser_prev = laser_now
        clusters_prev = clusters_now

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
            rospy.loginfo('[track_2d_lidar] process_laser missed its control loop. It actually takes %f secs.' % duration)
        r.sleep()

        # ========================================= loop =========================================

    # plot
    plt.show()


if __name__ == "__main__":
    rospy.init_node('test_coarse_level', anonymous=True)      
    realtime_test()
    rospy.spin()