#!/usr/bin/env python

import rospy, math, random
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates

# Parameters
vel_max = 1.0
vel_min = 0.1

# Define parameters, if parameter server provides no definitions
# Lower bound of the obstacle movement in y-direction
if rospy.has_param("pos_lb_y"):
  pos_lb_y = rospy.get_param("pos_lb_y")
else:
  pos_lb_y = 0.5

# Upper bound of the obstacle movement in y-direction
if rospy.has_param("pos_ub_y"):
  pos_ub_y = rospy.get_param("pos_ub_y")
else:
  pos_ub_y = 5.5

# Lower bound of the obstacle movement in x-direction
if rospy.has_param("pos_lb_x"):
  pos_lb_x = rospy.get_param("pos_lb_x")
else:
  pos_lb_x = 3.5

# Upper bound of the obstacle movement in x-direction
if rospy.has_param("pos_ub_x"):
  pos_ub_x = rospy.get_param("pos_ub_x")
else:
  pos_ub_x = 5.5

# Define global Twist message which is modified in the callback_base_pose_ground_truth and published in the main loop
Twist_msg = Twist()
ns = rospy.get_namespace()

# This function manages turnarounds and new (possibly random) velocities of the object
def callback(msg): 
  # Define random velocity if no specific velocity is defined in the parameter server
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  	vel_x = rospy.get_param("vel_x")
  else:
  	vel_y = random.uniform(vel_min, vel_max)
  	vel_x = random.uniform(vel_min, vel_max)

  # Find the index of the racecar
  try:
      arrayIndex = msg.name.index(ns[1:-1]+'::base_footprint')  # e.g. ns = '/robot_1/' 
  except ValueError as e:
      # Wait for Gazebo to startup
      pass
  else:
      # Extract our current position information
      # Turn in y-direction
      if msg.pose[arrayIndex].position.y >= pos_ub_y:
        Twist_msg.linear.y = -1.0*vel_y
      if msg.pose[arrayIndex].position.y <= pos_lb_y:
        Twist_msg.linear.y = vel_y
      # Turn in x-direction
      if msg.pose[arrayIndex].position.x >= pos_ub_x:
        Twist_msg.linear.x = -1.0*vel_x
      if msg.pose[arrayIndex].position.x <= pos_lb_x:
        Twist_msg.linear.x = vel_x


# This function initializes the mover node and publishes continously a Twist message
def move_object():
  rospy.init_node("Mover")
  r = rospy.Rate(10)
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('/gazebo/link_states', LinkStates, callback, queue_size=1)

  start_t = rospy.get_time()

  # initialize movement with random direction
  if rospy.has_param("vel_y"):
  	vel_y = rospy.get_param("vel_y")
  else:
  	vel_y = random.uniform(vel_min, vel_max)
  Twist_msg.linear.y = float(random.choice(['-1', '1'])) * vel_y

  if rospy.has_param("vel_x"):
  	vel_x = rospy.get_param("vel_x")
  else:
  	vel_x = random.uniform(vel_min, vel_max)
  Twist_msg.linear.x = float(random.choice(['-1', '1'])) * vel_x

  # publish movement command continuously
  while not rospy.is_shutdown():
    pub.publish(Twist_msg)
    r.sleep()


if __name__ == '__main__': 
  try:
    move_object()
  except rospy.ROSInterruptException:
    pass