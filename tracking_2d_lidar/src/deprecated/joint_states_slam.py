import numpy as np

class JointStates():
    '''
    Important: Values assigned here are all needed to be tuned.
    '''
    def __init__(self):
        # covariance matrix
        '''
        xs, xt, xb, xp, xc
        '''
        self.P = np.eye(6) * 0.01 # initialize with xs, xc


        # sensor
        self.xs = np.array([[0],  # alpha: x
                            [0],  # beta: y
                            [0]]) # psi: theta (rad.)

        # vehicle state covariance matrix
        self.V = np.array([[0.01, 0],   # v: velocity
                           [0, 0.01]])  # theta: steering angle

        # sensor to car
        self.xc = np.array([[0],  # delta alpha: x
                            [0],  # delta beta: y
                            [0]]) # delta psi: theta

        # static background
        '''
        Never change order, only remove and append.
        When removed (appended), be sure to remove (append) corresponding part of the covariance matrix.
        '''
        self.xb = []

        # dynamic track 
        '''
        Never change order, only remove and append.
        When removed (appended), be sure to remove (append) corresponding part of the covariance matrix.
        '''
        self.xt = []
        self.xp = []
        self.xt_counter = []  # -1: already mature
        self.xt_id = []  # the id will not change for the same track (test only)

        # cluster to tracks (or static background)
        self.matched_track_indices = []  # index of the matched track, -1: static background
        
        