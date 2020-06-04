import numpy as np
np.random.seed(237)
import matplotlib.pyplot as plt
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt.plots import plot_convergence, plot_evaluations, plot_objective

from objective_func_target_params import objective_func
import time
import pickle

# params
dim1 = Real(name='min_size', low=1, high=10)
dim2 = Real(name='inflation_size', low=5, high=15)
dim3 = Real(name='static_threshold', low=0.5, high=0.9)
dim4 = Real(name='speed_threshold', low=0.05, high=0.5)
dimensions = [dim1, dim2, dim3, dim4]

# objective func.
# @use_named_args(dimensions=dimensions)
# def f(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold):
#     return -1.0* objective_func(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold)  # maximize to minimize

@use_named_args(dimensions=dimensions)
def f(min_size, inflation_size, static_threshold,speed_threshold):
    return -1.0* objective_func(min_size, inflation_size, static_threshold, speed_threshold)  # maximize to minimize


# optimization
res = gp_minimize(f,                        # the function to minimize
                  dimensions=dimensions,    # the bounds on each dimension of x
                  acq_func="EI",            # the acquisition function
                  n_calls=1000,               # the number of evaluations of f
                  n_random_starts=10,        # the number of random initialization points
                  #x0=[5,8,0.1],                    # initial points
                  noise=0.2**2,             # the noise level (variance)(optional)
                  random_state=1234)        # the random seed

# print
print("Best fitness:", res.fun)
print("Best parameters:", res.x)

# write to pickle file
time_ = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/res/'+time_+'.pkl', 'wb') as file_:
    pickle.dump(res, file_)

# plot
fig, ax = plt.subplots()
ax = plot_convergence(res)
ax = plot_evaluations(res)
ax = plot_objective(res)
plt.show()

