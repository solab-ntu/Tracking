import matplotlib.pyplot as plt
from scipy.optimize import OptimizeResult
import pickle

def plot_1d(res):
    X = [x[0] for x in res.x_iters]
    Y = res.func_vals
    fig, ax = plt.subplots()

    best_x = res.x[0]
    best_f = res.fun
    ax.plot([best_x], [best_f], marker='*', markersize=20, color="red")


    ax.scatter(X, Y)

    

    plt.xlabel('x')
    plt.ylabel('f')

    # label
    for i, (x,y) in enumerate(zip(X,Y)):

        label = i

        plt.annotate(label, # this is the text
                    (x,y), # this is the point to label
                    textcoords="offset points", # how to position the text
                    xytext=(0,10), # distance from text to points (x,y)
                    ha='center') # horizontal alignment can be left, right or center    

    plt.show()


# test only
if __name__ == "__main__":
    #res = OptimizeResult()
    filename = 'test2.pkl'
    with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/res/'+filename, 'r') as f: # open in readonly mode
        res = pickle.load(f)
        plot_1d(res)