import numpy as np
np.random.seed(237)
import matplotlib.pyplot as plt
from skopt.acquisition import gaussian_ei

def plot_1d(res):
    plt.rcParams["figure.figsize"] = (6, 4)

    # Plot GP(x) + contours
    x = np.linspace(0, 0.5, 400).reshape(-1, 1)
    x_gp = res.space.transform(x.tolist())

    gp = res.models[-1]
    y_pred, sigma = gp.predict(x_gp, return_std=True)

    plt.plot(x, y_pred, "g--", label=r"$\mu_{GP}(x)$")
    plt.fill(np.concatenate([x, x[::-1]]),
            np.concatenate([y_pred - 1.9600 * sigma,
                            (y_pred + 1.9600 * sigma)[::-1]]),
            alpha=.2, fc="g", ec="None")

    # Plot sampled points
    plt.plot(res.x_iters,
            res.func_vals,
            "r.", markersize=15, label="Observations")

    plt.title(r"$x^* = %.4f, f(x^*) = %.4f$" % (res.x[0], res.fun))
    plt.legend(loc="best", prop={'size': 8}, numpoints=1)
    plt.grid()

    plt.show()