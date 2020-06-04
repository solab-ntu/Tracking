#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt

def feedback_callback(data):
  global foo

  if not data: # empty
    foo = [[0,0,0]]
    return
  foo.append(data.data)
  
  
def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('vx variance')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('vy variance')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()

  
  
def velocity_plotter():
  global foo
  rospy.init_node("visualize_time_series", anonymous=True)
  
  topic_name = "/kf_test"
  rospy.Subscriber(topic_name, Float32MultiArray, feedback_callback, queue_size = 1) # define feedback topic here!

  # two subplots sharing the same t axis
  fig, (ax_v, ax_omega) = plt.subplots(2, sharex=True)
  plt.ion()
  plt.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    t = []
    v = []
    omega = []
    
    for fooo in foo:
      t.append(fooo[0])
      v.append(fooo[1])
      omega.append(fooo[2])
          
    plot_velocity_profile(fig, ax_v, ax_omega, np.asarray(t), np.asarray(v), np.asarray(omega))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    foo = [[0,0,0]]
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass

