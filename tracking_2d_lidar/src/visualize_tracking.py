import matplotlib.pyplot as plt
import rospy
import time
from nav_msgs.msg import MapMetaData

class VisualizeTracking():
    """ Matplotlib wrapper to visualize data (e.g. laser scan).
    """

    def __init__(self, x, map):  # map is the ProcessKF object
        # state
        self.x = x
        
        # init plot
        fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title('2D LiDAR Tracking', fontsize=22)
        #self.ax.plot([0], [0], marker='>', markersize=20, color="blue")  # plot the origin
        
        # get the size of the map
        x = map.map_info.origin.position.x
        y = map.map_info.origin.position.y
        w = map.map_info.width * map.map_info.resolution
        h = map.map_info.height * map.map_info.resolution
        self.ax.axis([x,w+x,y,h+y])

        # plot the map as background        
        # map_no_pad = map.map[map.pad_size:-map.pad_size, map.pad_size:-map.pad_size]
        # self.ax.imshow(map_no_pad, extent=[x,w+x,y,h+y], origin='lower', cmap='gray_r', vmin=0, vmax=255, interpolation='none')

    def plot(self):
        ax_list = []

        for i, track in enumerate(self.x.tracks):
            # id = i
            id = track.id

            # kf.x
            if track.static == 0:  # 0: dynamic, 1: static_map, 2: static object (unknown to the map)
                _x = track.kf.x[0][0]
                _y = track.kf.x[1][0]
                dx = (track.kf.x[2][0])*5  # enlarge the arrow length
                dy = (track.kf.x[3][0])*5
                a = self.ax.scatter(_x, _y, s=75.0, c='b', linewidths=0)
                ax_list.append(a)
                a = self.ax.text(_x-0.2, _y-0.2, str(id), color='b', fontsize=18)
                ax_list.append(a)
                a = self.ax.arrow(_x, _y, dx, dy, width=0.05, head_width=0.05, color='b')
                ax_list.append(a)

            # # xt - arrow
            # _x = track.xt[0][0]
            # _y = track.xt[1][0]
            # dx = (track.xt[2][0])*3  # enlarge the arrow length
            # dy = (track.xt[3][0])*3
            # a = self.ax.arrow(_x, _y, dx, dy, width=0.05, head_width=0.05, color='k')
            # ax_list.append(a)

            # id & counter 
            if track.counter < 0:  
                color = 'g'  # mature
                text = ('%d(%s)' % (id, track.static))
            else:
                color = 'r'  # tentative
                text = ('%d(%s)' % (id, track.static))

            # xp- box
            left = min(track.xp[0])
            bottom = min(track.xp[1])
            width = abs(max(track.xp[0])-left)
            height = abs(max(track.xp[1])-bottom)
            p = plt.Rectangle((left, bottom), width, height, fill=False, edgecolor=color, linewidth=20)
            #p.set_transform(self.ax.transAxes)
            #p.set_clip_on(False)
            a = self.ax.add_patch(p)
            ax_list.append(a)
            
            # xt - text
            top = bottom + height + 0.1
            a = self.ax.text(left, top, text, color=color, fontsize=18)
            ax_list.append(a)

            # xp - measurements
            a = self.ax.scatter(track.xp[0], track.xp[1], s=25.0, c='k', linewidths=0)
            ax_list.append(a)


        # plt.savefig('/home/wuch/Pictures/laser_box.png')
        
        # plot
        plt.pause(1e-12)  # pause for real time display
        for a in ax_list:  # clear data on the plot
            a.remove()

    def show(self):
        plt.show()