import numpy as np
import matplotlib.pyplot as plt

# rate & map
sim_time = 0.5
dt = 0.025
map_resolution = 0.05  # m
fig = plt.figure()
ax = fig.gca()
ax.set_aspect('equal')
arrow = None

def one_traj(x, y, w, vx, vy, vw, c, m, x_shift=0):
    global arrow
    ax = fig.gca()
    xs = [x]
    ys = [y]
    ws = [w]
    x_shifts = [x + x_shift * np.cos(w)]
    y_shifts = [y + x_shift * np.sin(w)]

    # generate legend
    dx = 0.015 * np.cos(w)
    dy = 0.015 * np.sin(w)
    arrow = plt.arrow(x,y,dx,dy,width=0.004,head_width=0.015, head_length=0.015, color=c
        ,label=r'$v_x$=%.2f, $v_y$=%.2f, $\omega$=%.2f' % (vx,vy,vw))
    if x_shift != 0:
        x_shifted = x + x_shift * np.cos(w)
        y_shifted = y + x_shift * np.sin(w)
        arrow = plt.arrow(x_shifted,y_shifted,dx,dy,width=0.004,head_width=0.015, head_length=0.015, color=c, alpha=0.3
        ,label=r'$v_x$=%.2f, $v_y$=%.2f, $\omega$=%.2f' % (vx,vy,vw))

    # for each sample points
    for i in range(int(sim_time/dt)):
        x = x + (vx*np.cos(w)+vy*np.cos(np.pi/2.0+w)) * dt 
        x_shifted = x + x_shift * np.cos(w)
        y = y + (vx*np.sin(w)+vy*np.sin(np.pi/2.0+w)) * dt 
        y_shifted = y + x_shift * np.sin(w)
        w = w + vw * dt 

        xs.append(x)
        ys.append(y)
        ws.append(w)
        x_shifts.append(x_shifted)
        y_shifts.append(y_shifted)

        # plot arrow
        dx = 0.015 * np.cos(w)
        dy = 0.015 * np.sin(w)
        plt.arrow(x,y,dx,dy,width=0.004,head_width=0.015, head_length=0.015, color=c)

        if x_shift != 0:
            plt.arrow(x_shifted,y_shifted,dx,dy,
                width=0.004,head_width=0.015, head_length=0.015, color=c, alpha=0.3)

    # plot traj no heading
    ax.set_xlim(np.floor(min(xs)), np.ceil(max(xs)))
    ax.set_xticks(np.arange(np.floor(min(xs))-map_resolution, np.ceil(max(xs))+map_resolution, map_resolution))
    ax.set_yticks(np.arange(np.floor(min(ys))-map_resolution, np.ceil(max(ys))+map_resolution, map_resolution))
    plt.plot(xs,ys,marker=m, color=c, markersize=10
        ,label=r'$v_x$=%.2f, $v_y$=%.2f, $\omega$=%.2f' % (vx,vy,vw))
    
    
    if x_shift != 0:
        plt.scatter(x_shifts, y_shifts, c=c, marker=m, s=100, alpha=0.3) 
    

# --- user settings

# # initial position
# x = 0.0
# y = 0.0
# w = 0.0  # heading in rad.

# # const velocity
# vx = 1.0
# vy = 1.5
# vw = 1.0  # angular vel

# important: large one should draw later

# one_traj(0., 0., np.pi/2.0, 0.3/sim_time, 0.84/sim_time,-0.5, 'r', 'x',0.15)
# one_traj(0., 0., np.pi/2.0, 1.0, 1.5, 1.0, 'b', 'o',0.15)
one_traj(0., 0., np.pi/2.0, 1.0, 1.5, 1.0, 'b', 'o',0.15)

# --- main



plt.grid()
plt.legend([arrow,], ['My label',])
plt.legend(loc='lower left')
plt.show()