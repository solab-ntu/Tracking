import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.animation as animation

fig = plt.figure()


# ims is a list of lists, each row is a list of artists to draw in the
# current frame; here we are just animating one artist, the image, in
# each frame
ims = []
for i in range(114):
    img=mpimg.imread('/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/imgs/'+ str(i+1) + '.png')
    im = plt.imshow(img, animated=True)
    ims.append([im])

ani = animation.ArtistAnimation(fig, ims, interval=1000, blit=True,
                                repeat_delay=None)

# ani.save('dynamic_images.mp4')

plt.show() 