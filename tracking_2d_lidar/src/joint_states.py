import numpy as np
import rospy
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise, Q_continuous_white_noise
from scipy.linalg import block_diag

from publish_path_object import PubPath

class State:
    def __init__(self, rate, use_displacement = False):  # use_displacement is deprecated
        # rate
        self.dt = 1.0/rate
        self.use_displacement = use_displacement

        # cluster to tracks (follow the cluster's order)
        self.track_indices = []  # index of the matched track

        # store Tracks
        self.tracks = []  

        # measurement variance
        self.var_range = 6.76e-6
        # self.var_range = 1e-4
        self.var_bearing = 1.579e-4

    def append(self, cluster, laser_now):
        # id
        max_id = 0
        for track in self.tracks:
            if track.id > max_id:
                max_id = track.id 
        the_id = max_id + 1

        t = Track(self.use_displacement, the_id)

        # xp
        x_i = laser_now.cartesian[cluster,0]
        y_i = laser_now.cartesian[cluster,1]
        t.xp = np.array([list(x_i), 
                        list(y_i)])

        # xt
        t.xt = np.array([[np.mean(x_i)],  # x
                         [np.mean(y_i)],  # y  
                         [0.],             # vx
                         [0.]])            # vy
  
        # xt_counter
        t.counter = 1  # count start from 1


        # kf
        dt = self.dt
        t.kf.x = np.array([[np.mean(x_i)],  # x
                           [np.mean(y_i)],  # y  
                           [0.],             # vx
                           [0.]])            # vy      # initial state (location and velocity)

        t.kf.P = np.array([[0.1, 0., 0., 0.],
                            [0., 0.1, 0., 0.],
                            [0., 0., 2., 0.],
                            [0., 0., 0., 2.]])                 # covariance matrix

        t.kf.F = np.array([[1., 0., dt, 0.],
                           [0., 1., 0, dt],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])    # state transition matrix
        
        if self.use_displacement:  # use displacement/dt as vel mesurement
            t.kf.H = np.array([[1., 0., 0., 0.],  # Measurement function
                               [0., 1., 0., 0.],    
                               [0., 0., 1., 0.],  
                               [0., 0., 0., 1.]])    
            
            t.kf.R = np.array([[0.01, 0., 0., 0.],
                               [0., 0.01, 0., 0.],
                               [0., 0., 0.1, 0.],
                               [0., 0., 0., 0.1]])                      # state uncertainty
        
        else:
            t.kf.H = np.array([[1., 0., 0., 0.],
                               [0., 1., 0., 0.]])    # Measurement function
        
            t.kf.R = np.array([[0.1, 0.],
                               [0., 0.1]])                      # state uncertainty

        #q = Q_discrete_white_noise(dim=2, dt=dt, var=2.0) # process uncertainty      
        q = Q_continuous_white_noise(dim=2, dt=dt, spectral_density=5.0) # process uncertainty        
        t.kf.Q = block_diag(q, q)  # assume the noise in x and y are independent

        # append the track
        self.tracks.append(t)

    def remove(self, indices_new):
        ''' Remove those tracks that their indices are "not" in indices_new. And updated the track_indices_new (indexed by clusters)
        Input:
            indices_new: new track indices that cluster_now corresponds to.
        Output:
            indices_updated: since there are tracks removed, the indices need to be updated. e.g. [12,1,5,9,7,5] to [4,0,1,3,2,1]
        '''
        # get the complementary indices 
        sorted_i = sorted(indices_new)
        total_i = np.arange(len(self.tracks))
        mask = np.zeros(total_i.shape,dtype=bool)
        mask[sorted_i] = True
        rest_i = list(total_i[~mask])
        
        # delete tracks that their indices are not in indices_new
        for i in reversed(rest_i):  # delete from the back so that the index is correct
            del self.tracks[i]

        # update indices_new (e.g. [12,1,5,9,7,5] to [4,0,1,3,2,1])
        _index = 0
        for i in range(max(indices_new)+1):
            someone_matched_flag = False
            for _i, i_new in enumerate(indices_new):
                if i_new == i:
                    indices_new[_i] = _index
                    someone_matched_flag = True
            if someone_matched_flag:
                _index += 1

        return indices_new


class Track:
    def __init__(self, use_displacement, the_id):       
        '''
        xp: measured position at every instant. updated without kalman filter.
        xt: np.array([[0],      # x: np.mean(xp[0]) updated without kalman filter
                      [0],      # y: np.mean(xp[1]) updated without kalman filter  
                      [0],      # vx: frame by frame difference
                      [0]])     # vy: frame by frame difference
      
        kf.x: np.array([[0],      # x: np.mean(xp[0]) updated with kalman filter
                        [0],      # y: np.mean(xp[1]) updated with kalman filter  
                        [0],      # vx
                        [0]])     # vy
        '''
        self.xp = None
        self.xt = None
        self.counter = None  # -1: already mature
        self.id = the_id  # the id will not change for the same track
        self.static = 0  # 0: dynamic, 1: static_map, 2: static object (unknown to the map)

        self.displacement = np.array([0., 0.])  # avg. displacement from point to point

        if use_displacement:
            self.kf = KalmanFilter(dim_x=4, dim_z=4)  # dimension of state, measurement
        else:
            self.kf = KalmanFilter(dim_x=4, dim_z=2)  # dimension of state, measurement

        self.car_frame_bound = (0., 0., 0., 0.)  # left right bottom top
        self.car_frame_state = None  # (x, y, vx, vy)
        
        # for /scan_static topic in process_kf.py
        self.static_count = 0  # -1: dynamic 

        # for test
        self.chi = None


    
        