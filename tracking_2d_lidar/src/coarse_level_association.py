#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import copy
from icp import icp
from joint_states import Track

# test only
import rospy
from laser_sub_fake import LaserSubFake
from euclidean_minimum_spanning_tree import EMST
from clustering import Cluster
from laser_subscriber import LaserSub

class CoarseLevel:
    def __init__(self, laser_now, laser_prev, clusters, clusters_prev, 
                    joint_state, tentative_threshold=5, icp_max_dist=0.5):
        # members
        self.x  = joint_state  # joint state
        self.tent_thresh = tentative_threshold  # tentative count thereshold

        # execute coarse level data association
        self.coarse_level_association(laser_now, laser_prev, clusters, clusters_prev, icp_max_dist)


    def coarse_level_association(self, laser_now, laser_prev, clusters_now, clusters_prev, icp_max_dist):
        
        # if cluster_now is not the first cluster
        if laser_prev is not None and clusters_prev is not None:
            # ICP (points match to points)
            dst_indices, T = icp(laser_now.cartesian, laser_prev.cartesian, 
                                init_pose=None, max_iterations=20, tolerance=0.001, max_dist=icp_max_dist)
            
            # Match Clusters (clusters match to clusters)
            matched_indices, avg_displacements = match_clusters(dst_indices, laser_now, laser_prev, clusters_now, clusters_prev, self.x.use_displacement)

            # Update Tracks's States
            track_indices_new = []
            for i, (cluster, match_i, d) in enumerate(zip(clusters_now.clusters, matched_indices, avg_displacements)):

                # coarse-level associated clusters (clusters that matched to previous)
                if match_i >= 0:
                    t_i = self.x.track_indices[match_i]  # track's index
                    track_indices_new.append(t_i)  # save this track index

                    # save displacement information
                    self.x.tracks[t_i].displacement = d  # x, y

                    # tentative
                    if self.x.tracks[t_i].counter > 0:
                        self.x.tracks[t_i].counter += 1

                        # check mature
                        if self.x.tracks[t_i].counter > self.tent_thresh:
                            self.x.tracks[t_i].counter = -1
        

                # coarse-level unassociated clusters append tentative track
                elif match_i == -1:
                    # append track
                    self.x.append(cluster, laser_now)

                    # append track_indices
                    track_indices_new.append(len(self.x.tracks) - 1)  # new index


            # remove xt, xp who is not matched by any cluster_now. update indices
            track_indices_new = self.x.remove(indices_new=track_indices_new)

            # update track_indices
            self.x.track_indices = track_indices_new

        # the first time to recieve a cluster
        else:  
            for i, cluster in enumerate(clusters_now.clusters):
                # append track
                self.x.append(cluster, laser_now)

                # init track indices
                self.x.track_indices.append(i)


def match_clusters(dst_indices, laser_now, laser_prev, clusters_now, clusters_prev, use_displacement):
    ''' Using ICP and clustering result to match cluster to cluster 
    Input:
        dst_indices: matching indices of destination (previous) points for each source (current) point. Provided by icp. (Cluster.set_arr & Laser.cartesian follow the same ordering) 
        clusters_now: current (source) clusters
        clusters_prev: previous (destination) clusters 
    Output:
        matched_b_indices: matching indices of destination (previous) clusters for each source (current) cluster. (-1: no mathing cluster)
    '''
    a = clusters_now
    b = clusters_prev
    matched_b_indices = []
    avg_displacements = []  # displacement between matched clusters

    # for each cluster in clusters_now
    for i, a_root in enumerate(a.roots):  # this root represent a cluster
        
        # for all the element in the cluster, find those that their roots are this root
        b_roots = []  # collect matched point's root
        a_indices = []
        b_indices = []

        for j, i_root in enumerate(a.set_arr):
            
            if i_root == a_root and dst_indices[i] >= 0:
                b_index = dst_indices[j]  # matched b index
                b_root = b.set_arr[b_index]
                b_roots.append(b_root)
                a_indices.append(j)
                b_indices.append(b_index)

        # there are matching points for this cluster
        if b_roots != []:
            best_b_root = most_frequent(b_roots)
            
            # get displacements
            displacement = np.array([0.0, 0.0])
            if use_displacement:
                count_d = 0
                for b_root, b_i, a_i in zip(b_roots, b_indices, a_indices):
                    if b_root == best_b_root:
                        displacement += laser_now.cartesian[a_i] - laser_prev.cartesian[b_i]
                        count_d += 1
                displacement = displacement/float(count_d)
            
            # record this matchiing
            try:  # if matched_dst_root is a cluster smaller than Cluster.min_size, it can not be found in the Cluster.roots
                matched_b_index = b.roots.index(best_b_root)  # index of the cluster (clusters.roots has the same ordring as clusters.clusters)
            except:
                matched_b_indices.append(-1)
                avg_displacements.append(-1)
            else:
                # we only allow one to one matching
                if matched_b_index in matched_b_indices:  # if someone else is matched
                    exist_i = matched_b_indices.index(matched_b_index)
                    # if i am bigger
                    if len(a.clusters[i]) > len(a.clusters[exist_i]):
                        matched_b_indices[exist_i] = -1  # someone will be loser
                        avg_displacements[exist_i] = -1
                        matched_b_indices.append(matched_b_index)  # i won
                        avg_displacements.append(displacement)
                    else:  
                        matched_b_indices.append(-1)  # i lose
                        avg_displacements.append(-1)      
                else:
                    matched_b_indices.append(matched_b_index)  
                    avg_displacements.append(displacement)

        
        # there is no matching point for this cluster
        else:
            matched_b_indices.append(-1)
            avg_displacements.append(-1)

    return matched_b_indices, avg_displacements


def most_frequent(List):
    return max(set(List), key = List.count)


def test_icp():
    '''Test the icp result from two laser scans.
    '''
    # read two scans
    a = LaserSubFake('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/bagfiles/test_icp_1.dat')  # source (current)
    b = LaserSubFake('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/bagfiles/test_icp_0.dat')  # destination (previous)

    # plot icp point-to-point
    #test_plot_icp_point2point(a, b)

    # plot icp cluster-to-cluster
    test_plot_icp_cluster2cluster(a, b)


# def test_plot_icp_point2point(a, b):
#     ''' Deprecated
#     Need to change icp code (last few lines) in order to work
#     '''
#     #
#     A = a.cartesian
#     B = b.cartesian

#     # ICP
#     dst_indices, T, distances, iterations = icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001)
#     A_matched = A[src_indices,:]
#     B_matched = B[dst_indices,:]

#     # transform A according to T
#     m = A.shape[1]
#     A_homo = np.ones((m+1,A.shape[0]))  
#     A_homo[:m,:] = np.copy(A.T)
#     A_homo_after = np.dot(T, A_homo)
#     A_after = A_homo_after[:m,:].T

#     # print results
#     print('Homogeneous Transformation Matrix: ' + str(T))
#     print('Number of iterations: %f' % iterations)

#     # plot setup
#     plt.figure()
#     plt.title('Result of Iterative Closest Point')

#     # plot scan
#     plt.scatter(A[:,0], A[:,1], c='r', linewidths=0, s=300)
#     plt.scatter(B[:,0], B[:,1], c='g', linewidths=0, s=300)
#     plt.scatter(A_after[:,0], A_after[:,1], c='b', linewidths=0, s=300)
#     plt.scatter(A_matched[:,0], A_matched[:,1], c='w', linewidths=0, s=100)
#     plt.scatter(B_matched[:,0], B_matched[:,1], c='w', linewidths=0, s=100)

#     # plot lines between matching points
#     for a, b in zip(A_matched, B_matched):
#         x = a[0]
#         y = a[1]
#         dx = b[0] - a[0]
#         dy = b[1] - a[1]
#         plt.arrow(x, y, dx, dy, width=0.0001, head_width=0.005)

#     plt.show()


def test_plot_icp_cluster2cluster(laser_a, laser_b):
    # source: a.
    mst_a = EMST(laser_a)
    clusters_a = Cluster(mst_a, k=1.0, min_size=10)

    # destination: b.
    mst_b = EMST(laser_b)
    clusters_b = Cluster(mst_b, k=1.0, min_size=10)

    # icp
    dst_indices, T = icp(laser_a.cartesian, laser_b.cartesian, init_pose=None, max_iterations=20, tolerance=0.001)
    matched_cluster_indices, _ = match_clusters(dst_indices, laser_a, laser_b, clusters_a, clusters_b)

    # plot
    plt.figure()
   
    # colors to iterate over
    colors_a = ['r','y']*20
    colors_b = ['b','g']*20

    # plot source clusters and arrows to destination
    count = 1
    for cluster, color, cluster_b_index in zip(clusters_a.clusters, colors_a, matched_cluster_indices): 
        plt.scatter(laser_a.cartesian[cluster,[0]], laser_a.cartesian[cluster,[1]], s=10.0, c=color, linewidths=0)
        if cluster_b_index >= 0: 
            # plot arrow points to the corresponding cluster
            x1 = np.mean(laser_a.cartesian[cluster,[0]])
            y1 = np.mean(laser_a.cartesian[cluster,[1]])
            cluster_b = clusters_b.clusters[cluster_b_index]
            x2 = np.mean(laser_b.cartesian[cluster_b, [0]])
            y2 = np.mean(laser_b.cartesian[cluster_b, [1]])
            dx = x2 - x1
            dy = y2 - y1
            plt.arrow(x1, y1, dx, dy, width=0.001, head_width=0.05)
            plt.text((x1+x2)/2.0, (y1+y2)/2.0, str(count), fontsize=18)
            count += 1
    
    # plot destination clusters
    for cluster, color in zip(clusters_b.clusters, colors_b):
        plt.scatter(laser_b.cartesian[cluster,[0]], laser_b.cartesian[cluster,[1]], s=10.0, c=color, linewidths=0)

    plt.show()


def realtime_test():
    ''' Internal test.
    '''
    # laser sub
    laser = LaserSub()

    # init loop
    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():  # wait until first scan is received so that laser is initialized
        if laser.first_recieved_done:  # laser specs initialized by ROS sensor_msgs/LaserScan message
            break
        else:
            rospy.loginfo('[coarse_level_association] Waiting for laser to init.')
        r.sleep()

    laser_prev = None
    clusters_prev = None
    clusters_id = None

    # init plot
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Clusters Tracking', fontsize=18)
    ax.axis([-5.0,5.0,-5.0,5.0])
    ax.plot([0], [0], marker='>', markersize=20, color="red")  # plot the origin
    

    # loop
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        # --- 1. laser input (copy self.laser at this moment)
        laser_now = copy.deepcopy(laser)

        # --- 2. EMST (create input graph for segmentation) 
        mst = EMST(laser_now)

        # --- 3. EGBIS (apply image segmentation technique to clustering)
        clusters_now = Cluster(mst, k=1.0, min_size=10)
        if clusters_id is None:  # first set of clusters generated
            clusters_id = [i for i in range(len(clusters_now.clusters))]

        # --- 4. ICP
        if laser_prev is not None:
            dst_indices, T = icp(laser_now.cartesian, laser_prev.cartesian, init_pose=None, max_iterations=20, tolerance=0.001)
            matched_cluster_indices, _ = match_clusters(dst_indices, laser_now, laser_prev, clusters_now, clusters_prev)

            # --- 5. Visualize
            laser_a = laser_now
            laser_b = laser_prev
            clusters_a = clusters_now
            clusters_b = clusters_prev
            clusters_id_new = []

            # colors to iterate over
            colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']*20

            # plot source clusters with the same id as the matched destination clusters
            ax_list = []
            for cluster, color, cluster_b_index in zip(clusters_a.clusters, colors, matched_cluster_indices): 
                a = ax.scatter(laser_a.cartesian[cluster,[0]], laser_a.cartesian[cluster,[1]], s=20.0, c=color, linewidths=0)
                ax_list.append(a)
                if cluster_b_index >= 0: 
                    # show id
                    x1 = np.mean(laser_a.cartesian[cluster,[0]])
                    y1 = np.mean(laser_a.cartesian[cluster,[1]])
                    cluster_b = clusters_b.clusters[cluster_b_index]
                    x2 = np.mean(laser_b.cartesian[cluster_b, [0]])
                    y2 = np.mean(laser_b.cartesian[cluster_b, [1]])
                    _id = clusters_id[cluster_b_index]
                    clusters_id_new.append(_id)
                    a = ax.text((x1+x2)/2.0, (y1+y2)/2.0, str(_id), fontsize=22)
                    ax_list.append(a)
                else:  # no cluster_b match me.
                    _id = max(clusters_id) + 1
                    clusters_id_new.append(_id)
            
            clusters_id = clusters_id_new
            
            # plot
            plt.pause(1e-12)  # pause for real time display
            for a in ax_list:  # clear data on the plot
                a.remove()

        # end of loop
        laser_prev = laser_now
        clusters_prev = clusters_now
        r.sleep()

    # plot
    plt.show()

# if __name__ == "__main__":
#     # 1. test icp and cluster matching with two static LaserFake
#     #test_icp()

#     # 2. test icp and cluster matching with real lidar
#     rospy.init_node('test_coarse_level', anonymous=True)      
#     realtime_test()
#     rospy.spin()
    