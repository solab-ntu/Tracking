import numpy as np
from run_launch_file import launch
import csv
import yaml
import pickle
import time
import matplotlib.pyplot as plt

x1 = []
y1 = []
x2 = []
y2 = []

# read test (tracking) and label (ground true)
# filename_1 = 'history_measure_1'
# filename_2 = 'history_measure_2'
# filename_1 = 'history_kf_1'
filename_1 = 'history_kf_14'
filename_2 = 'label_ok'
with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/outputs/'+filename_1+'.csv') as testfile:
    with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/outputs/'+filename_2+'.csv') as labelfile:

        rows_1 = csv.reader(testfile)
        rows_2 = csv.reader(labelfile)

        x_test = []
        y_test = []
        x_label = []
        y_label = []

        for i, (test_row, label_row) in enumerate(zip(rows_1, rows_2)):
            if i%2 == 0:  # collect x (in a row)
                x_test = [float(a) for a in test_row]
                x_label = [float(a) for a in label_row]
            else:  # collect y (in the next row of x)
                y_test = [float(a) for a in test_row]
                y_label = [float(a) for a in label_row]

                # ------ now at i/2 th frame. main work here: ------
                x1 += x_test
                x2 += x_label
                y1 += y_test
                y2 += y_label

            
# plot
for i, (x,y) in enumerate(zip(x1,y1)):
    if x<100 and y<100:    
        alpha = 0.099 + 0.9/len(x1)*i
        plt.scatter(x, y, c='r', s=160, alpha=alpha, label='1')

for i, (x,y) in enumerate(zip(x2,y2)):
    if x<100 and y<100:    
        alpha = 0.099 + 0.9/len(x2)*i
        plt.scatter(x, y, c='b', s=60, alpha=alpha, label='2')

plt.title(filename_1+' '+filename_2)

plt.show()
                