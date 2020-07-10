#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

cov = None

def amcl_cb(data):
    global cov
    cov = data.pose.covariance

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)

    pub = rospy.Publisher('robot_pose', PoseWithCovarianceStamped, queue_size=1)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.pose.covariance = cov
        p.pose.pose.position.x = trans[0]
        p.pose.pose.position.y = trans[1]
        p.pose.pose.position.z = trans[2]
        p.pose.pose.orientation.x = rot[0]
        p.pose.pose.orientation.y = rot[1]
        p.pose.pose.orientation.z = rot[2]
        p.pose.pose.orientation.w = rot[3]
        pub.publish(p)

        rate.sleep()