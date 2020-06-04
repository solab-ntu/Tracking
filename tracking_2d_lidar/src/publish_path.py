#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import time

start_time = time.time()

path = Path()
path2 = Path()

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)

def amcl_cb(data):
    if time.time() < 20:
        pass
    else:
        global path2
        path2.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        path2.poses.append(pose)
        path2_pub.publish(path2)

rospy.init_node('path_node')

# in stage simulaton
odom_sub = rospy.Subscriber('/robot_0/odom', Odometry, odom_cb)

# in real life
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)

# publish
path_pub = rospy.Publisher('/path_odom', Path, queue_size=10)
path2_pub = rospy.Publisher('/path_amcl', Path, queue_size=10)


# # wait and publish goal
# goal_pub = rospy.Publisher('/robot_0/move_base_simple/goal', PoseStamped, queue_size=10)
# goal = PoseStamped()
# goal.header.frame_id = 'map'
# goal.pose.position.x = 24.0
# goal.pose.position.y = 3.0
# goal.pose.orientation.w = 1.0

# while time.time()-start_time < 23.5:
#     pass

# goal_pub.publish(goal)

if __name__ == '__main__':
    rospy.spin()