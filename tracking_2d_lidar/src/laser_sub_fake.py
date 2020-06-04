#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserSubFake():
    """ Subscribe to laserscan topic.
    """

    def __init__(self, data_file=None):      
        # laser specs. 
        self.angle_min = -2.09234976768  # in rad.
        self.angle_max = 2.09234976768  # in rad.
        self.angle_increment = 0.00613592332229  # in rad.
        self.laser_total_num = int(abs(self.angle_max-self.angle_min)/float(self.angle_increment)) + 2  # 683

        self.range_min = 0.02
        self.range_max = 5.6
        
        # read "a" scan data
        if not data_file:
            data_file = '/home/wuch/tracking_ws/src/Tracking/tracking_2d_lidar/bagfiles/test_data_1.dat'
        with open(data_file, 'r') as f:
            self.ranges = f.read().split(",")
        self.ranges = [float(i) for i in self.ranges]  # convert string to float

        # np.array of 2D cartesian coordinate points (dimention: #_of_measurements x 2)
        self.cartesian = None
        # create cartesian 2d coordinate points (x,y)
        self.polar_to_cartesian()  # convert polar to cartesian
        
        # check if the first message is received in order to initialize laser specs 
        self.first_recieved_done = True

    def polar_to_cartesian(self):
        # the heading of the lidar is toward x axis
        laser_non_nan_num = 0  # the number of non-nan ranges

        cartesian = np.zeros((self.laser_total_num, 2))

        for i,range in enumerate(self.ranges):
            if range == range:  # The usual way to test for a foat('nan') is to see if it's equal to itself
                current_angle = self.angle_min + i*self.angle_increment
                if current_angle >= -1*np.pi/2.0 or current_angle <= np.pi/2.0:
                    x = range * np.cos(current_angle)
                else:
                    x = range * -1 * np.cos(current_angle)

                y = range * np.sin(current_angle)
                
                cartesian[laser_non_nan_num][0] = x
                cartesian[laser_non_nan_num][1] = y

            laser_non_nan_num += 1
        
        np.resize(cartesian, (laser_non_nan_num,2))  # without nans
        self.cartesian = cartesian

# test only
# if __name__ == "__main__":
#     a = LaserSubFake()
