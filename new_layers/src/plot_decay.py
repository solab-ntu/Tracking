import numpy as np
import matplotlib.pyplot as plt

a = 252
y_buff = 125
length = 3.0
x_delay = length/3.0
factor = 15
num=100

x = np.linspace(0.0, length/0.05, num=num)
y = y_buff + (a - y_buff) * np.exp(-1.0 * x /float(factor))

length = 1.5
x2 = np.linspace(0.0, length/0.05, num=num)
y2 = y_buff + (a - y_buff) * np.exp(-1.0 * x /float(factor))

#
# x_front_num = int(num/(length/x_delay))
# y_front = np.array([a]*x_front_num)
# y_end = y[:(num-x_front_num)]
# y = np.concatenate([y_front,y_end])

plt.plot(x,y)
plt.plot(x2, y2)
plt.ylim(0.0, a+5)
plt.show()