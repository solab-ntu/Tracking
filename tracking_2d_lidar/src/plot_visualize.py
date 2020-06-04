import matplotlib.pyplot as plt

#---  1. laser input
def scatter_laser(laser):  # laser: a LaserSub object
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')
    # plot the origin point (0,0)
    ax.plot([0], [0], marker='>', markersize=10, color="red")

    # plot
    ax.scatter(laser.cartesian[:,0], laser.cartesian[:,1], s=10.0, c='b', linewidths=0)
    
    plt.savefig('/home/wuch/Pictures/laser_raw.png')

# --- 2. EMST
def plot_mst(mst, laser):  # tree: a EMST object, laser: a LaserSub object
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    # plot the origin point (0,0)
    ax.plot([0], [0], marker='>', markersize=10, color="red")

    # plot
    for t in mst.result_tree:
        e, va, vb = t
        va_x = laser.cartesian[va][0]
        va_y = laser.cartesian[va][1]
        vb_x = laser.cartesian[vb][0]
        vb_y = laser.cartesian[vb][1]
        lines = ax.plot([va_x, vb_x], [va_y, vb_y], c='b')

    plt.savefig('/home/wuch/Pictures/laser_emst.png')
    

# --- 3. EGBIS
def plot_segmentation(clusters, laser):
    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    # plot the origin point (0,0)
    ax.plot([0], [0], marker='>', markersize=10, color="red")


    # colors to iterate over
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']*10

    # plot
    for cluster, color in zip(clusters.clusters, colors): 
        a = ax.scatter(laser.cartesian[cluster,[0]], laser.cartesian[cluster,[1]], s=10.0, c=color, linewidths=0)
    
    plt.savefig('/home/wuch/Pictures/laser_seg.png')
