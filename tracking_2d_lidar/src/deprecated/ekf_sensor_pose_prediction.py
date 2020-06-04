import rospy
import numpy as np
import time

class SensorPosePrediction:
    def __init__(self, wheelbase):
        while wheelbase==None or wheelbase<=0:
            rospy.loginfo('[track_2D_lidar] wheelbase not specified or being negative. Please terminate.')
        self.L = wheelbase

    def initialize(self):
        self.last_time = time.time()

    def run(self, odom, x):
        ''' Predict sensor pose according to odometry input
        Input:
            odom: nav_msgs.msg.Odometry
            x: joint_states.JointStates
        '''
        # # test only
        # _time = time.time()

        # G (eqn. 14)
        rad = x.xs[2][0] - x.xc[2][0]
        R = np.array([[np.cos(rad), -1.0*np.sin(rad)],[np.sin(rad), np.cos(rad)]])
        ba = np.array([[x.xc[1][0]],[-1.0*x.xc[0][0]]])
        _G = np.concatenate((R.dot(np.array([[1.0],[0.0]])), R.dot(ba)), axis=1)
        G = np.concatenate((_G, np.array([[0.0, -1.0]])), axis=0)

        # (eqn. 16)
        Psc = x.P[[[0],[1],[2],[-3],[-2],[-1]],[0,1,2,-3,-2,-1]]

        # from now on: delta time dependent
        now = time.time()
        delta_t = now - self.last_time
        self.last_time = now

        # xs' (eqn. 11)
        delta_l = odom.v * delta_t
        tL = np.tan(odom.theta) / float(self.L)
        delta_psi = delta_l * tL
        foo = delta_psi*ba + np.array([[delta_l],[0]])
        ab = x.xs[:2] + R.dot(foo)         
        psi = x.xs[2] - delta_psi
        psi = psi.reshape((-1,1))  # np.array([3]) to np.array([[3]])
        x.xs = np.concatenate((ab, psi), axis=0)

        # P' (eqn. 16)
        sec2 = 1.0/np.cos(odom.theta)**2
        U = delta_t* np.array([[1.0, 0.0],[tL, odom.v*sec2/self.L]])
        Q = U.dot(x.V.dot(U.T))

        poo = delta_psi*x.xc[:2] + np.array([[0.0], [delta_l]])
        _Fs = np.concatenate((np.eye(2), R.dot(poo)), axis=1)
        Fs = np.concatenate((_Fs, np.array([[0.0,0.0,1.0]])), axis=0)

        _Fc1 = delta_psi*R.dot(np.array([[0.0,1.0],[-1.0,0.0]]))
        _Fc2 = np.concatenate((_Fc1, -1.0*R.dot(poo)), axis=1)
        Fc = np.concatenate((_Fc2, np.zeros((1,3))), axis=0)

        F = np.concatenate((Fs, Fc), axis=1)
        
        x.P[:3,:3] = F.dot(Psc.dot(F.T)) + G.dot(Q.dot(G.T))
        x.P[3:,:3] = np.concatenate((x.P[3:,:3], x.P[3:,-3:]), axis=1).dot(F.T)
        x.P[:3,3:] = F.dot(np.concatenate((x.P[:3,3:], x.P[-3:,3:]), axis=0))

        # # test only
        # duration = time.time() - _time
        # rospy.loginfo(duration)
        # rospy.loginfo('--')


  