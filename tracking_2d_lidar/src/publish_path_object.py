#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PubPath():

    def __init__(self, id):

        rospy.init_node('dyn_'+str(id)+'_path_node')

        self.path = Path()

        # publish
        self.pub = rospy.Publisher('/dyn'+str(id)+'/path', Path, queue_size=10)

        # run
        self.run()

    def update(self, x, y):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        self.path.poses.append(pose)


    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.pub.publish(self.path)
            r.sleep()




# if __name__ == '__main__':
    