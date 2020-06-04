#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from math import sqrt

length = 0
flag = True

def cb(data):
    rospy.loginfo('First data recieved')
    global length
    global flag  # only do once

    last_x = 0
    last_y = 0

    if flag:
        for pose in data.poses:
            if last_x == 0 and last_y ==0:
                last_x = pose.pose.position.x
                last_y = pose.pose.position.y
                continue
            
            dx = pose.pose.position.x - last_x
            dy = pose.pose.position.y - last_y
            last_x = pose.pose.position.x
            last_y = pose.pose.position.y
            this_len = sqrt(dx**2+dy**2)
            length += this_len

        flag = False
            
        rospy.loginfo(length)

# node
rospy.init_node('listen_to_path')

# in stage simulaton
rospy.loginfo('haha listen to me')
sub = rospy.Subscriber('/path_amcl', Path, cb)


if __name__ == '__main__':
    rospy.spin()