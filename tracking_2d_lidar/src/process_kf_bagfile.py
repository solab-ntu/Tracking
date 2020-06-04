import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tracking_2d_lidar.msg import TracksMsg, TracksArrayMsg
import copy
import matplotlib.pyplot as plt

class ProcessKF:
    def __init__(self, state, inflate_size=8, static_threshold=0.7, speed_threshold=0.2, prediction_time=3.0):
        # state
        self.x = state
        
        # map param (for static map object detect)
        self.inflate_size = inflate_size   # unit: pixel
        self.pad_size = inflate_size * 2   # should be no smaller than the inflate_size (unit: pixel)
        self.static_threshold = static_threshold  # proportion of a cluster that overlap with inflated static map
        
        # speed threshold
        self.speed_threshold = speed_threshold

        # sub to map to judge who is static
        try:
            msg = rospy.wait_for_message('/map', OccupancyGrid)  # subscribe once
            self.map_info = msg.info
            self.map = self.inflate_map(msg, self.inflate_size)
        except:
            rospy.loginfo('[process_kf] topic /map not recieved. Please shutdown.')

        # pub tracks x,y,vx,vy to costmap
        rospy.set_param('/new_layers/predict_layer/prediction_time', prediction_time)   
        self.pub_tracks = rospy.Publisher('/tracks', TracksArrayMsg, queue_size=1)

        # for visualization to compare with true vel
        self.pub = rospy.Publisher('/robot_1/vel_kf', Twist, queue_size=1)

        # test kf
        self.pub_kf_state = rospy.Publisher('/kf_state', Odometry, queue_size=1)

        # history
        self.x_history = []
        self.y_history = []

        # a laser scan only contains static obstacles
        #self.static_scan = LaserScan()
        self.static_scan = None
        self.pub_static_scan = rospy.Publisher('/scan_static', LaserScan, queue_size=1)


    def inflate_map(self, msg, inflate_size):
        h = msg.info.height
        w = msg.info.width
        m = np.reshape(np.array(msg.data), (h, w))  # the map data, in row-major order, starting with (0,0).  
                                                    # Occupancy probabilities are in the range [0,100].  Unknown is -1.
        m_new = np.zeros(m.shape)
        # inflate
        for i, cell in np.ndenumerate(m):
            if cell > 0:  # occupied (free_space == 0)
                r = i[0]
                c = i[1]
                r_bottom = r-inflate_size
                r_top = r+inflate_size+1
                c_right = c-inflate_size
                c_left = c+inflate_size+1

                if r_bottom < 0:
                    r_bottom = 0
                if r_top > h:
                    r_top = h
                if c_right < 0:
                    c_right = 0
                if c_left > w:
                    c_left = w
                
                m_new[r_bottom:r_top, c_right:c_left] = 60
        
        # pad
        m_new = np.pad(m_new,((self.pad_size,self.pad_size),(self.pad_size,self.pad_size)),'constant',constant_values = (90,90))  # measurements may fall out of the map
                

        return m_new

    def check_static(self, x , y):  # check how many of these points are static points
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        r = (y - origin_y)/self.map_info.resolution + self.pad_size
        c = (x - origin_x)/self.map_info.resolution + self.pad_size
        r = r.astype(int)
        c = c.astype(int)

        try:
            num = (self.map[list(r), list(c)] > 0).sum()
        except:
            num = len(list(r))  # measurement out of map (with padding) bound: consider static

        return num

    def calc_R(self, track_size, x_car, y_car, odom_phi, amcl_cov):
        theta = np.arctan(y_car/x_car)
        rho = np.sqrt(x_car**2 + y_car**2)  # rho 
        H = np.array([[np.cos(theta), -rho*np.sin(theta)], 
                    [np.sin(theta), rho*np.cos(theta)]])
        R = np.array([[self.x.var_range, 0.], [0., self.x.var_bearing]])
        Rr = H.dot(R.dot(H.T))
        R_rotation = np.array([[np.cos(odom_phi), -np.sin(odom_phi)], [np.sin(odom_phi), np.cos(odom_phi)]]) 
        Rw = R_rotation.dot(Rr.dot(R_rotation.T)) + amcl_cov
        #Rc = Rw/float(track_size)
        #return Rc
        return Rw

    def predict_update(self, laser_odom_now, clusters_now, rate):
        '''
        xp, xt: just replace with new matched ones
        kf: predict and update with Kalman filter
        '''
        # world frame to car frame
        def world_to_car(x, y):  
            centroid_x = (x - laser_odom_now.odom_position[0])
            centroid_y = (y - laser_odom_now.odom_position[1])
            x_world = centroid_x * np.cos(-laser_odom_now.odom_position[2]) - centroid_y * np.sin(-laser_odom_now.odom_position[2])
            y_world = centroid_x * np.sin(-laser_odom_now.odom_position[2]) + centroid_y * np.cos(-laser_odom_now.odom_position[2])
            
            return x_world, y_world

        def world_to_car_vel(x, y):  
            vx = x * np.cos(-laser_odom_now.odom_position[2]) - y * np.sin(-laser_odom_now.odom_position[2])
            vy = x * np.sin(-laser_odom_now.odom_position[2]) + y * np.cos(-laser_odom_now.odom_position[2])
            
            return vx, vy

        
        assert len(self.x.track_indices) == len(clusters_now.clusters)

        # record for baysian optimization
        x_history = []  
        y_history = []

        # laser scan that contains only static obstacles
        self.static_scan = laser_odom_now.laser_data

        # for each track
        for i, track in enumerate(self.x.tracks):
            xp_i = np.array([[],[]])
            xp_car_i = np.array([[],[]])

            static_count = 0  # how many points in this cluster is static point on the map
            
            # find the cluster that matches this track (clusters to track)
            this_cluster = None
            # assert len(laser_odom_now.cartesian_car_frame[:,0]) == len(laser_odom_now.cartesian[:,0])
            for cluster, track_i in zip(clusters_now.clusters, self.x.track_indices):
                if track_i == i:
                    # collect points
                    x_i = laser_odom_now.cartesian[cluster,0]
                    y_i = laser_odom_now.cartesian[cluster,1]
                    p_i = np.array([list(x_i), 
                                    list(y_i)])
                    xp_i = np.concatenate((xp_i, p_i), axis=1)

                    # collect points in car frame
                    x_car_i = laser_odom_now.cartesian_car_frame[cluster,0]
                    y_car_i = laser_odom_now.cartesian_car_frame[cluster,1]
                    p_car_i = np.array([list(x_car_i), 
                                        list(y_car_i)])
                    xp_car_i = np.concatenate((xp_car_i, p_car_i), axis=1)

                    # collect points in car frame
                    left = laser_odom_now.cartesian_car_frame[cluster,0].min()
                    right = laser_odom_now.cartesian_car_frame[cluster,0].max()
                    bottom = laser_odom_now.cartesian_car_frame[cluster,1].min()
                    top = laser_odom_now.cartesian_car_frame[cluster,1].max()
                    track.car_frame_bound = (left, right, bottom, top)

                    # check whether they are static points on the map
                    static_count += self.check_static(x_i, y_i)
                    this_cluster = cluster

            if self.x.use_displacement:
                xp_prev = copy.deepcopy(track.xp)

            # save the collect
            track.xp = xp_i

            # static or dynamic part1 (static map overlay check)
            track_size = xp_i.shape[1]
            if static_count/float(track_size) > self.static_threshold:  # over 70 precent are static points on the map 
                track.static = 1  # 0: dynamic, 1: static_map, 2: static object (unknown to the map)
            else:  
                track.static = 0  # later speed_threshold will check whether it is actually 0 or 2 

            centroid_x = np.mean(track.xp[0])
            centroid_y = np.mean(track.xp[1])
            centroid_x_car = np.mean(xp_car_i[0])
            centroid_y_car = np.mean(xp_car_i[1])
            dt = 1.0/rate

            # # a. xt (frame by frame)
            # dx = (centroid_x - track.xt[0])/dt
            # dy = (centroid_y - track.xt[1])/dt
            # track.xt = np.array([[centroid_x],  # x
            #                      [centroid_y],  # y  
            #                      [dx],        # vx
            #                      [dy]])       # vy

            # b. kf 
            # get measurement z   
            if self.x.use_displacement:
                # TO DO: if no matching the displacement will be -1
                z = np.array([[centroid_x],   # x (centroid)
                            [centroid_y],     # y (centroid)
                            [track.displacement[0]/dt],    # vx (displacement)
                            [track.displacement[1]/dt]])   # vy (displacement)
            else:
                z = np.array([[centroid_x], 
                            [centroid_y]])

            # update R according to observation model
            R = self.calc_R(track_size, centroid_x_car, centroid_y_car, laser_odom_now.odom_position[2], laser_odom_now.amcl_pose_cov)
            track.kf.R = R

            # kf predict and update
            track.kf.predict()
            track.kf.update(z)

            # # static or dynamic part2 (chi-squared test)
            # if track.static != 1:  # only test those that do not overlap with static map
            #     self.chi_squared_test(track)
            # static or dynamic part2 (speed threshold)
            if track.static != 1:  # 0: dynamic, 1: static_map, 2: static object (unknown to the map)
                if (track.kf.x[2][0]**2 + track.kf.x[3][0]**2) < self.speed_threshold**2:
                    track.static = 2
                else:
                    track.static = 0

            #track.chi = np.sqrt(track.kf.x[2,0]**2 + track.kf.x[3,0]**2)  # test only

            # change back to car frame for visualization (and hand-labeling)
            if track.static == 0:
                # car_frame_state: for visualization in car frame
                x_updated_car, y_updated_car = world_to_car(track.kf.x[0][0], track.kf.x[1][0])  # change to car frame
                vx, vy = world_to_car_vel(track.kf.x[2][0], track.kf.x[3][0])  # change to car frame
                track.car_frame_state = (x_updated_car, y_updated_car, vx, vy) 

                # append history
                x_history.append(x_updated_car)  # kf updated
                y_history.append(y_updated_car)
                # x_history.append(centroid_x_car)  # measurement
                # y_history.append(centroid_y_car)
            
            # save static stuff laser scans
            if track.static == 0:  # make dynamic objects's range = nan
                # assert len(laser_odom_now.valid_indices) == len(laser_odom_now.cartesian[:,0])
                valid_i = np.array(laser_odom_now.valid_indices)[this_cluster]
                ranges_arr = np.array(list(self.static_scan.ranges))
                ranges_arr[valid_i] = np.nan  # ranges (in polar coordinate) has the same index ordering as cartesian
                self.static_scan.ranges = tuple(ranges_arr)
            else:  # when static
                if track.static_count == -1:  # previous frame is dynamic
                    track.static_count = 1
                elif track.static_count > 0 and track.static_count<=5:  # still consider as dynamic for 5 consecutive frames
                    track.static_count += 1
                    valid_i = np.array(laser_odom_now.valid_indices)[this_cluster]
                    ranges_arr = np.array(list(self.static_scan.ranges))
                    ranges_arr[valid_i] = np.nan  # ranges (in polar coordinate) has the same index ordering as cartesian
                    self.static_scan.ranges = tuple(ranges_arr)

        if x_history != []:
            self.x_history.append(x_history)
            self.y_history.append(y_history)
        else:
            # objective_function.py will ignore 1000,1000
            self.x_history.append([1000])
            self.y_history.append([1000])


    def chi_squared_test(self, t):
        vx_vy = t.kf.x[2:4,0:1]
        P = t.kf.P[2:4,2:4]

        # compare with zero velocity
        #chi_squared = (np.zeros((2,1))-vx_vy).T.dot(np.linalg.inv(P).dot(np.zeros((2,1))-vx_vy))
        chi_squared = ((-1.0 * vx_vy).T.dot(np.linalg.inv(P)).dot(-1.0 * vx_vy))
        t.chi = chi_squared
        # if not reject the hypoph
        if chi_squared <= 5.991:  # confidence: 95% (alpha=0.05), dof:2
            t.static = True


    def publish_to_costmap(self):       
        msg_arr = TracksArrayMsg()

        msg_arr_ = []

        for track in self.x.tracks:
            if track.static == 0 and track.counter < 0:  # dynamic and mature
                msg = TracksMsg()
                msg.x.data = track.kf.x[0][0]
                msg.y.data = track.kf.x[1][0]
                msg.vx.data = track.kf.x[2][0]
                msg.vy.data = track.kf.x[3][0]

                msg_arr_.append(msg)

        msg_arr.tracks = msg_arr_

        self.pub_tracks.publish(msg_arr)


    def publish_static_scan(self):
        self.pub_static_scan.publish(self.static_scan)
        

    def pub_estimated_vel(self):
        '''
        for visulization to compare with true vel
        '''
        id_of_track = 1  # user defined

        try:
            vel = Twist()
            vel.linear.x = self.x.tracks[id_of_track].kf.x[2]
            vel.linear.y = self.x.tracks[id_of_track].kf.x[3]
            self.pub.publish(vel)
        except IndexError:
            rospy.loginfo('[process_kf] can not find the assigned track in pub_estimated_vel()')

    def publish_kf_state(self, laser_odom_now):
        #i = 0  # user defined track id
        for i, t in enumerate(self.x.tracks):
            if t.id == 7:

                a = Odometry()
                try:
                    self.x.tracks[i]
                except IndexError:
                    rospy.loginfo('[process_kf] can not find the assigned track id = %d' % i)
                else:
                    a.pose.pose.position.x = self.x.tracks[i].kf.x[0][0]
                    a.pose.pose.position.y = self.x.tracks[i].kf.x[1][0]

                    a.twist.twist.linear.x = self.x.tracks[i].kf.x[2][0]
                    a.twist.twist.linear.y = self.x.tracks[i].kf.x[3][0]

                    a.pose.covariance = list(self.x.tracks[i].kf.P.flatten()) + [0]*20  
                    # a.pose.covariance = list(self.x.tracks[i].kf.R.flatten())+ \
                    #     list(laser_odom_now.amcl_pose_cov.flatten()) + [0]*28  

                    self.pub_kf_state.publish(a)


    def plot_map(self):  # for debug
        fig = plt.figure()

        ax = fig.add_subplot(111)
        ax.set_title('colorMap')
        plt.imshow(self.map, origin='lower', cmap='gray_r', vmin=0, vmax=255, interpolation='none')
        ax.set_aspect('equal')
        plt.show()
        