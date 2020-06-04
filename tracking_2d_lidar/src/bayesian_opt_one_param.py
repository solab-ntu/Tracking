import numpy as np
np.random.seed(237)
import matplotlib.pyplot as plt
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt.plots import plot_convergence, plot_evaluations, plot_objective
import time
import pickle

from objective_func_one_param import objective_func
from read_bayesian_opt_res import plot_1d

# params
dim1 = Real(name='speed_threshold', low=0.001, high=0.5)
dimensions = [dim1]

# objective func.
# @use_named_args(dimensions=dimensions)
# def f(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold):
#     return -1.0* objective_func(k, min_size, icp_max_dist, inflation_size, static_threshold, speed_threshold)  # maximize to minimize

@use_named_args(dimensions=dimensions)
def f(speed_threshold):
    return -1.0* objective_func(speed_threshold)  # maximize to minimize


# optimization
res = gp_minimize(f,                        # the function to minimize
                  dimensions=dimensions,    # the bounds on each dimension of x
                  acq_func="EI",            # the acquisition function
                  n_calls=30,                # the number of evaluations of f
                  n_random_starts=5,        # the number of random initialization points
                  #x0=[0.1],                # initial points
                  noise=0.1**2,             # the noise level (variance)(optional)
                  random_state=1234)        # the random seed

# print
print("Best fitness:", res.fun)
print("Best parameters:", res.x)

# plot
fig, ax = plt.subplots()
ax = plot_convergence(res)
plt.show()

plot_1d(res)

# write to pickle file
time_ = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/res/'+time_+'.pkl', 'wb') as file_:
    pickle.dump(res, file_)



