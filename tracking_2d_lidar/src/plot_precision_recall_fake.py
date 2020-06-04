import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

frontier_x = []
frontier_y = []
precision = []
recall = []

# click event
def onclick(event):
    global frontier_x
    global frontier_y
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))
    frontier_x.append(event.xdata)
    frontier_y.append(event.ydata)

def onclick_2(event):
    global frontier_x
    global frontier_y
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))
    precision.append(event.xdata)
    recall.append(event.ydata)

# get data


# for filename in os.listdir('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/f1s/0414'):
#     with open(os.path.join('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/f1s/0414', filename), 'r') as f: # open in readonly mode
#         the_list = pickle.load(f)
#         precision.append(the_list[0])
#         recall.append(the_list[1])

# plot 1: wait for hand click to generate frontier
fig, ax = plt.subplots()
ax.set_xlabel('Recall', fontsize=20)
ax.set_ylabel('Precision', fontsize=20)
ax.set_xlim([0,1.1])
ax.set_ylim([0,1.1])
ax.scatter(recall, precision, s=80, label='Sampled Parameter Sets', alpha=0.5)
ax.text(0,0,'Please click the frontier points\nin sequence then\nclose the plot.', fontsize=30)
ax.set_aspect('equal')
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.tight_layout()
plt.show()

# plot 2: show the frontier
fig, ax = plt.subplots()
ax.set_title('Recall-Precision', fontsize=20)
ax.set_xlabel('Recall', fontsize=20)
ax.set_ylabel('Precision', fontsize=20)
ax.set_xlim([0,1.1])
ax.set_ylim([0,1.1])
# ax.scatter(recall, precision, marker='x', label='Sampled Parameter Sets')
ax.plot(frontier_x, frontier_y, marker='o', c='r', linewidth=2, label='Frontier')
ax.set_aspect('equal')
ax.legend(loc='lower left')
cid = fig.canvas.mpl_connect('button_press_event', onclick_2)

plt.tight_layout()
plt.show()


# plot 3: show everything
fig, ax = plt.subplots()
ax.set_title('Recall-Precision', fontsize=20)
ax.set_xlabel('Recall', fontsize=20)
ax.set_ylabel('Precision', fontsize=20)
ax.set_xlim([0,1.1])
ax.set_ylim([0,1.1])
ax.scatter(recall, precision, marker='x', label='Sampled Parameter Sets')
ax.plot(frontier_x, frontier_y, marker='o', c='r', linewidth=2, label='Frontier')
ax.set_aspect('equal')
ax.legend(loc='lower left')
cid = fig.canvas.mpl_connect('button_press_event', onclick_2)

plt.tight_layout()
plt.show()