#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import time

start_time =time.time()

rospy.init_node('path_node')

# wait and publish goal
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 5.0
goal.pose.position.y = 0.0
goal.pose.orientation.w = 1.0

while time.time()-start_time < 15:
    pass

goal_pub.publish(goal)

if __name__ == '__main__':
    rospy.spin()