#!/usr/bin/env python
import rospy
from track_2d_lidar import Track2DLidar
    
if __name__ == '__main__':
    rospy.init_node('track_2d_lidar', anonymous=True)
    
    tracker = Track2DLidar(process_odom_rate=10, process_laser_rate=3,
                            wheelbase=2.64)

    rospy.spin()