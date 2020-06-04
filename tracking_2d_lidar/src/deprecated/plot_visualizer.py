import matplotlib.pyplot as plt

class PlotVisual():
    """ Matplotlib wrapper to visualize data (e.g. laser scan).
    """

    def __init__(self, axis):
        # 
        fig, self.axs = plt.subplots(2, 2)

        # titles
        fig.suptitle('2D LiDAR Scanning', fontsize=20)

        titles = ['Scanning',     # --- 1. 
        'Minimum Spanning Tree',  # --- 2.
        'Clustering',             # --- 3.
        '']

        # setup plot
        for ax, title in zip(self.axs.flatten(), titles):
            ax.axis(axis)
            ax.set_aspect('equal', adjustable='box')

            ax.set_title(title, fontsize=18)

            # plot the origin point (0,0)
            ax.plot([0], [0], marker='>', markersize=10, color="red")

    def visualize(self, laser, input_list):
        # things to be removed
        axs_list = []
        l_list = []

        # input
        laser_plot, mst, clusters = input_list  # every one needs laser so it is not here
        
        # --- 1. 
        if laser_plot:
            a = self.scatter_realtime_laser(laser)
            axs_list.append(a)
        # --- 2.
        if mst:
            lines_list = self.plot_mst(mst, laser)
            l_list.append(lines_list)
        # --- 3.
        if clusters:
            a_list = self.plot_segmentation_realtime(clusters, laser)
            axs_list += a_list

        # pause for real time display
        plt.pause(1e-12)

        # clear data on the plot
        for a in axs_list:
            a.remove()

        for lines_list in l_list:
            for lines in lines_list:
                lines.pop(0).remove()


    def plot_realtime(self, X, Y):
        a = self.axs[0,0].plot(X, Y)
        
        # pause for real time display
        plt.pause(1e-12)
        
        # clear data on the plot
        a.remove()

    def scatter_realtime(self, X, Y):
        a = self.axs[0,0].scatter(X, Y)
        
        # pause for real time display
        plt.pause(1e-12)
        
        # clear data on the plot
        a.remove()

    #---  1. laser input
    def scatter_realtime_laser(self, laser):  # laser: a LaserSub object
        ax = self.axs[0,0]

        # plot
        a = ax.scatter(laser.cartesian[:,0], laser.cartesian[:,1], s=10.0, c='b', linewidths=0)
        
        return a
    
    # --- 2. EMST
    def plot_mst(self, mst, laser):  # tree: a EMST object, laser: a LaserSub object
        ax = self.axs[0, 1]
        
        # plot
        lines_list = []

        for t in mst.result_tree:
            e, va, vb = t
            va_x = laser.cartesian[va][0]
            va_y = laser.cartesian[va][1]
            vb_x = laser.cartesian[vb][0]
            vb_y = laser.cartesian[vb][1]
            lines = ax.plot([va_x, vb_x], [va_y, vb_y], c='b')

            lines_list.append(lines)
        
        return lines_list

    # --- 3. EGBIS
    def plot_segmentation_realtime(self, clusters, laser):
        ax = self.axs[1, 0]

        # colors to iterate over
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']*10

        # plot
        ax_list = []
        for cluster, color in zip(clusters.clusters, colors): 
            a = ax.scatter(laser.cartesian[cluster,[0]], laser.cartesian[cluster,[1]], s=10.0, c=color, linewidths=0)
            ax_list.append(a)

        return ax_list 

    def show(self):
        plt.show()