#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserSub():
    """ Subscribe to laserscan topic.
    """

    def __init__(self):
        # check if the first message is received in order to initialize laser specs 
        self.first_recieved_done = False
        # laser specs. 
        self.angle_min = None  # in rad.
        self.angle_max = None  # in rad.
        self.angle_increment = None  # in rad.
        self.laser_total_num = None

        self.range_min = None
        self.range_max = None
        
        self.ranges = []

        # np.array of 2D cartesian coordinate points (dimention: #_of_measurements x 2)
        self.cartesian = None
        
        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
    def laser_callback(self, data):
        # setup specs (only need to be done once)
        if not self.first_recieved_done:
            self.angle_min = data.angle_min
            self.angle_max = data.angle_max
            self.angle_increment = data.angle_increment
            self.laser_total_num = int(abs(self.angle_max-self.angle_min)/float(self.angle_increment)) + 2  # 683

            self.range_min = data.range_min
            self.range_max = data.range_max
        
        # recieve new ranges
        self.ranges = data.ranges

        # create cartesian 2d coordinate points (x,y)
        self.polar_to_cartesian()  # convert polar to cartesian
    
    def polar_to_cartesian(self):
        # the heading of the lidar is toward x axis
        laser_non_nan_num = 0  # the number of non-nan ranges

        cartesian = np.zeros((self.laser_total_num, 2))

        for i,range in enumerate(self.ranges):
            if range == range:  # The usual way to test for a foat('nan') is to see if it's equal to itself
                
                # polar to cartesian
                current_angle = self.angle_min + i*self.angle_increment
                if current_angle >= -1*np.pi/2.0 or current_angle <= np.pi/2.0:
                    x = range * np.cos(current_angle)
                else:
                    x = range * -1 * np.cos(current_angle)

                y = range * np.sin(current_angle)
                
                # if too close to lidar: deprecate
                if abs(x) < 0.05 and abs(y) < 0.05:
                    continue

                # store 
                cartesian[laser_non_nan_num][0] = x
                cartesian[laser_non_nan_num][1] = y

                laser_non_nan_num += 1
        
        self.cartesian = np.resize(cartesian, (laser_non_nan_num,2))  # without nans and points too close to lidar

        # the first scan is recieved and its cartesian is done
        if not self.first_recieved_done:
            self.first_recieved_done = True