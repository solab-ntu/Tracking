#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class PlotEstimatedVel:
    def __init__(self):
        rospy.init_node('plot_estimated_vel', anonymous=True)

        # two subplots sharing the same t axis
        # self.fig, (self.ax_1, self.ax_2, self.ax_3, self.ax_4) = plt.subplots(4, sharex=True)
        self.fig, (self.ax_3, self.ax_4) = plt.subplots(2, sharex=True)
        plt.ion()
        #plt.show()
        self.r = 5  # plot rate
        self.time_count = 0

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

        self.vx_true = 0.
        self.vy_true = 0.

        self.kf_P = np.zeros((4,4))  # covariance matrix

        rospy.Subscriber('/kf_state', Odometry, self.kf_state_cb)  # 5Hz
        rospy.Subscriber('/car2/cmd_vel', Twist, self.cmd_vel_cb)  # 5Hz


    def r1_odom_cb(self, msg):
        self.odom = msg

    def kf_state_cb(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y =  msg.pose.pose.position.y 
        self.vx =  msg.twist.twist.linear.x 
        self.vy =  msg.twist.twist.linear.y 

        self.kf_P = np.asarray(msg.pose.covariance[0:16]).reshape(4,4)

    def cmd_vel_cb(self, msg):
        self.vx_true = msg.linear.x
        
    def run(self):
        r = rospy.Rate(self.r) # define rate here
        while not rospy.is_shutdown():
            self.plot(self.x, self.y, self.vx, self.vy)
            r.sleep()
        rospy.on_shutdown(self.save_fig)

    def save_fig(self):
        plt.savefig('/home/wuch/Pictures/haha.png')
   
    
    def plot(self, data_1=None, data_2=None, data_3=None, data_4=None):
        # # x-axis data
        # now = rospy.get_rostime()
        # self.t.append((now.secs-self.t_0.secs))
        self.t.append(self.time_count)
        self.time_count += 1.0/self.r

        # y-axis data
        self.data_1.append(data_1)
        self.data_2.append(data_2)
        self.data_3.append(data_3)
        self.data_4.append(data_4)

        # # variance frome statistic
        # self.data_1_var.append(np.var(self.data_1))
        # self.data_2_var.append(np.var(self.data_2))
        # self.data_3_var.append(np.var(self.data_3))
        # self.data_4_var.append(np.var(self.data_4))

        # variance frome statistic
        self.data_3_var.append(self.vx_true)
        self.data_4_var.append(self.vy_true)

        # # variance from kalman filter
        # self.data_1_kfvar.append(self.kf_P[0][0])
        # self.data_2_kfvar.append(self.kf_P[1][1])
        # self.data_3_kfvar.append(self.kf_P[2][2])
        # self.data_4_kfvar.append(self.kf_P[3][3])

        # # plot

        fontP = FontProperties()
        fontP.set_size('small')

        # # data_1
        # if data_1 != None:
        #     self.ax_1.cla()
        #     self.ax_1.grid()
        #     self.ax_1.set_ylabel(r'Estimated $x$')
        #     self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1), '-bx')
        #     # self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_var), '-r')
        #     # self.ax_1.plot(np.asarray(self.t), np.asarray(self.data_1_kfvar), '-g', linewidth=3)

        # # data_2
        # if data_2 != None:
        #     self.ax_2.cla()
        #     self.ax_2.grid()
        #     self.ax_2.set_ylabel(r'Estimated $y$')
        #     self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2), '-bx')
        #     # self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_var), '-r')
        #     # self.ax_2.plot(np.asarray(self.t), np.asarray(self.data_2_kfvar), '-g', linewidth=3)

        # data_3
        if data_3 != None:
            self.ax_3.cla()
            self.ax_3.grid()
            self.ax_3.set_ylabel(r'$v_x$'+' (m/s)', fontsize=14)
            # line_1 = self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3), '-bx', label='Estimated')[0]
            # line_2 = self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_var), '-r', label='True')[0]
            self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3), '-bx', label='Estimated')
            self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_var), '-r', label='True')
            # self.ax_3.plot(np.asarray(self.t), np.asarray(self.data_3_kfvar), '-g', linewidth=3)

            self.ax_3.legend(loc='upper right',
                         borderaxespad=0.1, prop=fontP)

        # data_4
        if data_4 != None:
            self.ax_4.cla()
            self.ax_4.grid()
            self.ax_4.set_ylabel(r'$v_y$'+' (m/s)', fontsize=14)
            self.ax_4.set_xlabel('time (s)', fontsize=14)
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4), '-bx', label='Estimated')
            self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_var), '-r', label='True')
            # self.ax_4.plot(np.asarray(self.t), np.asarray(self.data_4_kfvar), '-g', linewidth=3)

            # self.ax_4.set_ylim(-0.08,0.1)
            self.ax_4.legend(loc='upper right',
                         borderaxespad=0.1, prop=fontP)
        
        self.fig.canvas.draw()
        plt.tight_layout()
        #plt.legend()
        # Adjust the scaling factor to fit your legend text completely outside the plot
        # (smaller value results in more space being made for the legend)
        
        # plt.subplots_adjust(right=0.8)
        # self.fig.legend([line_1, line_2],
        #                 labels=['Estimated','True'],
        #                 loc='center right',
        #                 borderaxespad=0.1, prop=fontP)    # Small spacing around legend box


if __name__ == "__main__":
    node = PlotEstimatedVel()
    node.run()
    

    