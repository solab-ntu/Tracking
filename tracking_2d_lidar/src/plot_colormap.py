import numpy as np
import matplotlib.pyplot as plt

x = np.arange(100)
y = x
t = x
plt.scatter(x, y, c=t, cmap='viridis')
plt.colorbar()
plt.show()