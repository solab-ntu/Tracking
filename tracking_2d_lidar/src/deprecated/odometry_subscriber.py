#!/usr/bin/env python
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomSub():
    """ Subscribe to odometry topic.
    """

    def __init__(self, rate=10):
        # first recieved
        self.odom_recieved = False
        self.first_recieved_done = False

        # bicycle model for carlike vehicles
        self.v = None      # speed
        self.theta = None  # steering angle: mean angle of the front wheels
        
        # subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.twist_callback)
        
    def odom_callback(self, data):
        self.odom_recieved = True
        self.v = data.twist.twist.linear.x

    def twist_callback(self, data):
        ''' Compliant with Stage Simulator
        Some command interfaces (such as the stage simulator in car-like mode) 
        require a geometry_msgs/Twist, but with changed semantics. The angular 
        velocity (z-axis) is interpreted as steering angle. Note, changing the 
        semantics of a message is not preferred in general, better switch to the 
        ackermann_msgs interface if possible.
        '''
        self.theta = data.angular.z

        if self.odom_recieved:
            self.first_recieved_done = True