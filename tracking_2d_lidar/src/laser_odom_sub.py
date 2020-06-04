#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class LaserOdomSub():
    """ Subscribe to laserscan topic (in robot coordinate), robot odometry topic.
        Then make the laser measurements in world coordinate. 
    """

    def __init__(self, use_amcl):
        # check if the first message is received in order to initialize laser specs 
        self.first_cartesian_done = False

        # odom
        self.odom_position = None  # x, y, yaw
        self.amcl_pose_cov = None  # amcl pose covariance

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
        self.cartesian_car_frame = None

        # make sure that cartesian and cartesian_car_frame are both updated when tracking.py is using them
        self.updated = True
        self.is_copying = False

        # LaserScan object (we will produce a laserscan contains only static obstacles in process_kf.py)
        self.laser_data = None
        self.valid_indices = []  # indices of valid scans of LaserScan
        
        # subscriber
        if use_amcl:
            rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)  # use amcl localization as odom
        else:
            rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def amcl_callback(self, data):
        # quaternion to euler
        quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.odom_position = (data.pose.pose.position.x, data.pose.pose.position.y, euler[2])  # yaw = euler[2]
        
        # covariance matrix
        c = np.asarray(data.pose.covariance).reshape(6,6)
        self.amcl_pose_cov = c[0:2,0:2]

    def odom_callback(self, data):
        # quaternion to euler
        quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.odom_position = (data.pose.pose.position.x, data.pose.pose.position.y, euler[2])  # yaw = euler[2]
        
    def laser_callback(self, data):
        # setup specs (only need to be done once)
        if not self.first_cartesian_done:
            self.angle_min = data.angle_min
            self.angle_max = data.angle_max
            self.angle_increment = data.angle_increment
            self.laser_total_num = int(abs(self.angle_max-self.angle_min)/float(self.angle_increment)) + 2  # 683

            self.range_min = data.range_min
            self.range_max = data.range_max
        
        # recieve new ranges
        self.ranges = data.ranges

        # create cartesian 2d coordinate points (x,y)
        if self.odom_position is not None:
            self.polar_to_cartesian()  # convert polar to cartesian
        else:
            rospy.loginfo("[laser_odom_sub] Laser recieved. Waiting for odom or amcl_pose.")

        # save the object
        self.laser_data = data
    

    def polar_to_cartesian(self):
        # the heading of the lidar is toward x axis
        laser_non_nan_num = 0  # the number of non-nan ranges
        
        valid_indices = []

        cartesian = np.zeros((self.laser_total_num, 2))
        cartesian_car_frame = np.zeros((self.laser_total_num, 2))

        for i,range in enumerate(self.ranges):
            if range == range:  # The usual way to test for a foat('nan') is to see if it's equal to itself
                
                # work around for stage simulator bug
                if range >= self.range_max:
                    continue

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

                # car frame coordinate
                cartesian_car_frame[laser_non_nan_num][0] = x
                cartesian_car_frame[laser_non_nan_num][1] = y

                # change back to world coordinate
                odom_x, odom_y, theta = self.odom_position
                x_world = x*np.cos(theta) - y*np.sin(theta) + odom_x  # rotate then translate
                y_world = x*np.sin(theta) + y*np.cos(theta) + odom_y

                # add odom (change into world coordinate) and store 
                cartesian[laser_non_nan_num][0] = x_world 
                cartesian[laser_non_nan_num][1] = y_world 

                laser_non_nan_num += 1

                valid_indices.append(i)
        
        # assign when tracking.py is not copying
        self.updated = False
        if self.is_copying == False:
            self.cartesian = np.resize(cartesian, (laser_non_nan_num,2))  # without nans and points too close to lidar
            self.cartesian_car_frame = np.resize(cartesian_car_frame, (laser_non_nan_num,2))  # without nans and points too close to lidar
            self.valid_indices = valid_indices
        self.updated = True

        assert len(self.cartesian[:,0]) == len(self.cartesian_car_frame[:,0])

        # the first scan is recieved and its cartesian is done
        if not self.first_cartesian_done:
            self.first_cartesian_done = True