from bresenham import bresenham
# pip install bresenham
import numpy as np
import matplotlib.pyplot as plt

start = np.array([0,0])
#end = np.array([10, 14])
end = np.array([0.01,14])
thick = 10

plt.plot([start[0], end[0]], [start[1], end[1]], c='r')

m = -1.0*(end[0]-start[0])/(end[1]-start[1])
dx = thick/2.0/np.sqrt(1.0+m*m)
dy = dx * m

shift_list = list(bresenham(int(np.round(dx)), int(np.round(dy)), int(np.round(-dx)), int(np.round(-dy))))

for shift in shift_list:
    start_shifted = start + np.array(shift)
    end_shifted = end + np.array(shift)

    start_shifted = np.round(start_shifted)
    end_shifted = np.round(end_shifted)

    new_points = list(bresenham(int(np.round(start_shifted[0])),int(np.round(start_shifted[1])),int(np.round(end_shifted[0])),int(np.round(end_shifted[1]))))

    for point in new_points:
        plt.scatter(point[0], point[1], marker='s', s=80, c='k')


plt.show()