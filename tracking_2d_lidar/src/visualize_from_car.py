import numpy as np
import matplotlib.pyplot as plt
import rospy
import time
from nav_msgs.msg import MapMetaData
import pickle

class VisualizeFromCar():
    """ Matplotlib wrapper to visualize data (e.g. laser scan).
        Use car frame. 
           x
           ^
        y _|
    """

    def __init__(self, laser_odom, x):  # map is the ProcessKF object
        # state
        self.x = x
        
        # init plot
        fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title('2D LiDAR Tracking', fontsize=22)
        self.ax.plot([0], [0], marker='^', markersize=16, color="blue")  # plot the origin
        
        # set size of the fig
        r = laser_odom.range_max -0.5
        self.ax.axis([-r,r,-r,r])

        # count frame
        self.count = 0

        # reminder
        # self.ax.text(-4, -4, 'click on every frame', color='b', fontsize=30)


    def plot(self, laser_odom):  # odom is laser_odom_now
        # plot
        ax_list = []

        # a = self.ax.text(4, -4, self.count, color='r', fontsize=30)
        # self.count += 1
        # ax_list.append(a)

        # xp - measurements
        a = self.ax.scatter(-1*laser_odom.cartesian_car_frame[:,1], laser_odom.cartesian_car_frame[:,0], s=25.0, c='k', linewidths=0)
        ax_list.append(a)

        for i, track in enumerate(self.x.tracks):
            id = track.id
            # id = i

            if track.static == 0:  # 0: dynamic, 1: static_map, 2: static object (unknown to the map)
                # kf.x
                _x = track.car_frame_state[0]
                _y = track.car_frame_state[1]
                a = self.ax.scatter(-1*_y, _x, s=450.0, c='b', linewidths=0, alpha=0.5)  # * -1 is for matplotlib
                ax_list.append(a)
                vx = track.car_frame_state[2]
                vy = track.car_frame_state[3]
                a = self.ax.arrow(-1*_y, _x, -1*vy, vx, width=0.05, head_width=0.05, color='b')
                ax_list.append(a)


                # id & counter 
                if track.counter < 0:  
                    color = 'g'  # mature
                    # text = ('%d(%f)' % (id, track.chi))
                    text = ('%d' % (id))
                else:
                    color = 'r'  # tentative
                    # text = ('%d(%f)' % (id, track.chi)) 
                    text = ('%d' % (id)) 

                # xp- box   
                (left, right, bottom, top) = track.car_frame_bound
                height = abs(top-bottom)
                weight = abs(right-left)
                p = plt.Rectangle((-1*top, left), height, weight, fill=False, edgecolor=color, linewidth=2)
                #p.set_transform(self.ax.transAxes)
                #p.set_clip_on(False)
                a = self.ax.add_patch(p)
                ax_list.append(a)
                
                # xt - text
                a = self.ax.text(-1*top, right+0.1, text, color=color, fontsize=18)
                ax_list.append(a)

                
        
        # plot
        plt.tight_layout()
        plt.pause(1e-12)  # pause for real time display

        # savefig
        #plt.savefig('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/imgs/'+ str(self.count) + '.png')
        pickle.dump(self.ax, file('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/imgs/'+str(self.count) + '.pickle', 'w'))

        for a in ax_list:  # clear data on the plot
            a.remove()

    def show(self):
        plt.show()

    def close(self):
        plt.close()