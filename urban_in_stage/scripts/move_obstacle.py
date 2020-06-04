#!/usr/bin/env python

import rospy, math, random
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

# Define global Twist message which is modified in the callback_base_pose_ground_truth and published in the main loop
Twist_msg = Twist()

# whether robot_0 has moved 
robot_0_moved = False

# robots wait until robot_0
def callback(data):
  global robot_0_moved
  robot_0_moved = True

# This function initializes the mover node and publishes continously a Twist message
def move_object():
  global robot_0_moved
  rospy.init_node("Mover")
  r = rospy.Rate(10)
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('/robot_0/cmd_vel', Twist, callback)
  start_t = rospy.get_time()

  # initialize movement with random direction
  if rospy.has_param("vel_x"):
  	vel_x = rospy.get_param("vel_x")
  else:
  	vel_x = 10.0


  # publish movement command continuously
  while not rospy.is_shutdown():

    if robot_0_moved:
      #Twist_msg.linear.x = float(random.choice(['-1', '1'])) * vel_x
      Twist_msg.linear.x = vel_x
    else:
      Twist_msg.linear.x = 0.0

    pub.publish(Twist_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    move_object()
  except rospy.ROSInterruptException:
    pass