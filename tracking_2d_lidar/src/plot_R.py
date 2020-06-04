#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry

class EstimationError:
    def __init__(self):
        rospy.init_node('estimation_error', anonymous=True)

        # two subplots sharing the same t axis
        self.fig, (self.ax_1, self.ax_2, self.ax_4) = plt.subplots(3, sharex=True)
        plt.ion()
        plt.show()

        # x-axis data (time)
        self.t = []  
        self.t_0 = rospy.get_rostime()
        # y-axis data
        self.data_1 = []
        self.data_2 = []
        self.data_3 = []
        self.data_4 = []

        self.data_1_var = []
        self.data_2_var = []
        self.data_3_var = []
        self.data_4_var = []

        self.data_1_kfvar = []
        self.data_2_kfvar = []
        self.data_3_kfvar = []
        self.data_4_kfvar = []

        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.

        self.kf_R = np.zeros((4,4))  # covariance matrix
        self.amcl_cov = np.zeros((4,4))  # covariance matrix

        rospy.Subscriber('/kf_state', Odometry, self.kf_state_cb)  # 5Hz


    def r1_odom_cb(self, msg):
        self.odom = msg

    def kf_state_cb(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y =  msg.pose.pose.position.y 
        self.vx =  msg.twist.twist.linear.x 
        self.vy =  msg.twist.twist.linear.y 

        self.kf_R = np.asarray(msg.pose.covariance[0:4]).reshape(2,2)
        self.amcl_cov = np.asarray(msg.pose.covariance[4:8]).reshape(2,2)

        
    def run(self):
        r = rospy.Rate(2) # define rate here
        while not rospy.is_shutdown():
            self.plot(self.x, self.y, self.vx, self.vy)
            r.sleep()
        rospy.on_shutdown(self.myhook)

    def myhook(self):
        plt.savefig('/home/wuch/Pictures/haha.png')
   
    
    def plot(self, data_1=None, data_2=None, data_3=None, data_4=None):
        # x-axis data
        now = rospy.get_rostime()
        self.t.append(now.secs-self.t_0.secs)
        # y-axis data
        self.data_1.append(data_1)
        self.data_2.append(data_2)
        self.data_3.append(data_3)
        self.data_4.append(data_4)

        # variance frome statistic
        self.data_1_var.append(self.amcl_cov[0,0])
        self.data_2_var.append(self.amcl_cov[0,1])
        self.data_3_var.append(self.amcl_cov[1,0])
        self.data_4_var.append(self.amcl_cov[1,1])

        # variance from kalman filter
        self.data_1_kfvar.append(self.kf_R[0][0])
        self.data_2_kfvar.append(self.kf_R[0][1])
        self.data_3_kfvar.append(self.kf_R[1][0])
        self.data_4_kfvar.append(self.kf_R[1][1])

        # plot
        # data_1
        if data_1 != None:
            self.ax_1.cla()
            self.ax_1.grid()
            self.ax_1.set_ylabel(r'${\bf R}_{xx}$', fontsize=18)
            # self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1), '-bx')
            # self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_var), '-r', linewidth=3)
            self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_kfvar), '-b', linewidth=2)

        # data_2
        if data_2 != None:
            self.ax_2.cla()
            self.ax_2.grid()
            self.ax_2.set_ylabel(r'${\bf R}_{xy}={\bf R}_{yx}$', fontsize=18)
            # self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2), '-bx')
            # self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_var), '-r', linewidth=3)
            self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_kfvar), '-b', linewidth=2)

        # # data_3
        # if data_3 != None:
        #     self.ax_3.cla()
        #     self.ax_3.grid()
        #     self.ax_3.set_ylabel('vx')
        #     # self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3), '-bx')
        #     # self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_var), '-r')
        #     self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_kfvar), '-g', linewidth=3)

        # data_4
        if data_4 != None:
            self.ax_4.cla()
            self.ax_4.grid()
            self.ax_4.set_ylabel(r'${\bf R}_{yy}$', fontsize=18)
            self.ax_4.set_xlabel('time (s)', fontsize=18)
            # self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4), '-bx')
            # self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_var), '-r', linewidth=3)
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_kfvar), '-b', linewidth=2)
        # self.ax_1.set_ylim(0,0.02)
        # self.ax_2.set_ylim(-0.003,0.003)
        # self.ax_4.set_ylim(0,0.02)
        self.fig.canvas.draw()
        plt.tight_layout()

if __name__ == "__main__":
    node = EstimationError()
    node.run()
    

    