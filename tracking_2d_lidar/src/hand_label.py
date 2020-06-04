import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#import matplotlib.animation as animation
import csv
import pickle

'''
Important: for the current implementation, every frame needs at least one click. 
If the frame has no dynamic obstacle, please click the origin point (0,0). 
'''
count = 0
clicked = False

x_labels = []
y_labels = []

# click (Event handling and picking)
def onclick(event):
    global count
    global clicked
    clicked = True

    print('Frame %d: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        (count, event.button, event.x, event.y, event.xdata, event.ydata))

    x_labels.append(event.ydata)  # change matplotlib coordinate
    y_labels.append(-1*event.xdata)



# write clicks to file
with open('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/outputs/label.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)

    def onclose(event):
        global x_labels, y_labels
        writer.writerow(x_labels)
        writer.writerow(y_labels)

        x_labels = []
        y_labels = []

    # iterate over matplotlib objects
    for i in range(114):
        
        pickle.load(file('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/imgs/'+ str(i+1) + '.pickle'))

        fig = plt.gcf()
        cid = fig.canvas.mpl_connect('button_press_event', onclick)
        cid2 = fig.canvas.mpl_connect('close_event', onclose)
        plt.show()

        count += 1
        if clicked == False:  # warning
            print('[hand_label.py] Error: forgot to click. Please restart!')

        # user close window to enter next iteration

        





