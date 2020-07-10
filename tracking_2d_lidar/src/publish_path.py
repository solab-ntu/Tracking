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
    if time.time()-start_time < 10:
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

## in stage simulaton
# odom_sub = rospy.Subscriber('/robot_0/odom', Odometry, odom_cb)

# in real life
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)

# publish
path_pub = rospy.Publisher('/path_odom', Path, queue_size=10)
path2_pub = rospy.Publisher('/path_amcl', Path, queue_size=10)


if __name__ == '__main__':
    rospy.spin()