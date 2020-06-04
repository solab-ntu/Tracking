#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry


class EstimationError:
    def __init__(self):
        rospy.init_node('estimation_error', anonymous=True)
        self.odom = Odometry()

        # two subplots sharing the same t axis
        self.fig, (self.ax_1, self.ax_2, self.ax_3, self.ax_4) = plt.subplots(4, sharex=True)
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

        self.error_x = 0.
        self.error_y = 0.
        self.error_vx = 0.
        self.error_vy = 0.

        self.P = np.zeros((4,4))  # covariance matrix

        rospy.Subscriber('/robot_1/odom', Odometry, self.r1_odom_cb)  # >5Hz
        rospy.Subscriber('/kf_state', Odometry, self.kf_state_cb)  # 5Hz


    def r1_odom_cb(self, msg):
        self.odom = msg

    def kf_state_cb(self, msg):
        self.error_x = msg.pose.pose.position.x - self.odom.pose.pose.position.x
        self.error_y =  msg.pose.pose.position.y - self.odom.pose.pose.position.y
        self.error_vx =  msg.twist.twist.linear.x - self.odom.twist.twist.linear.x
        self.error_vy =  msg.twist.twist.linear.y - self.odom.twist.twist.linear.y

        self.P = np.asarray(self.odom.pose.covariance[0:16]).reshape(4,4)

        
    def run(self):
        r = rospy.Rate(2) # define rate here
        while not rospy.is_shutdown():
            self.plot(self.error_x, self.error_y, self.error_vx, self.error_vy)
            r.sleep()
    
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
        self.data_1_var.append(np.var(self.data_1))
        self.data_2_var.append(np.var(self.data_2))
        self.data_3_var.append(np.var(self.data_3))
        self.data_4_var.append(np.var(self.data_4))

        # variance from kalman filter
        self.data_1_kfvar.append(self.P[0][0])
        self.data_2_kfvar.append(self.P[1][1])
        self.data_3_kfvar.append(self.P[2][2])
        self.data_4_kfvar.append(self.P[3][3])

        # plot
        # data_1
        if data_1 != None:
            self.ax_1.cla()
            self.ax_1.grid()
            self.ax_1.set_ylabel('x error')
            self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1), '-bx')
            self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_var), '-r')
            self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_kfvar), '-g')

        # data_2
        if data_2 != None:
            self.ax_2.cla()
            self.ax_2.grid()
            self.ax_2.set_ylabel('y error')
            self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2), '-bx')
            self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_var), '-r')
            self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_kfvar), '-g')

        # data_3
        if data_3 != None:
            self.ax_3.cla()
            self.ax_3.grid()
            self.ax_3.set_ylabel('vx error')
            self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3), '-bx')
            self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_var), '-r')
            self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_kfvar), '-g')

        # data_4
        if data_4 != None:
            self.ax_4.cla()
            self.ax_4.grid()
            self.ax_4.set_ylabel('vy error')
            self.ax_4.set_xlabel('Time [s]')
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4), '-bx')
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_var), '-r')
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_kfvar), '-g')
        
        self.fig.canvas.draw()

if __name__ == "__main__":
    node = EstimationError()
    node.run()
    

    