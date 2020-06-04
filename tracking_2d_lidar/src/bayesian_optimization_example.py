import numpy as np
np.random.seed(237)
import matplotlib.pyplot as plt
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt.plots import plot_convergence

# params
dim1 = Real(name='a', low=-2.0, high=2.0)
dim2 = Real(name='b', low=-2.0, high=2.0)
dimensions = [dim1, dim2]

# objective func.
@use_named_args(dimensions=dimensions)
def f(a, b, noise_sigma=0.1):
    return np.sin(5 * a) * (1 - np.tanh(b ** 2))\
           + np.random.randn() * noise_sigma

# optimization
res = gp_minimize(f,                        # the function to minimize
                  dimensions=dimensions,    # the bounds on each dimension of x
                  acq_func="EI",            # the acquisition function
                  n_calls=30,               # the number of evaluations of f
                  n_random_starts=5,        # the number of random initialization points
                  #x0=[],                    # initial points
                  noise=0.2**2,             # the noise level (variance)(optional)
                  random_state=1234)        # the random seed

# print
print("Best fitness:", res.fun)
print("Best parameters:", res.x)

# plot
fig, ax = plt.subplots()
ax = plot_convergence(res)
plt.show()

