#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def median_filter(data, window_size):
    padded_data = np.pad(data, (window_size // 2, window_size // 2), mode='edge')
    filtered_data = np.array([np.median(padded_data[i:i+window_size]) for i in range(len(data))])
    return filtered_data

def lidar_callback(scan):
    ranges = np.array(scan.ranges)
    filtered_ranges = median_filter(ranges, window_size=5)
    
    # Eliminate out-of-range values
    filtered_ranges = np.where((filtered_ranges >= 0.1) & (filtered_ranges <= 3.5), filtered_ranges, float('inf'))
    
    scan.ranges = filtered_ranges
    pub.publish(scan)

rospy.init_node('lidar_filter')
sub = rospy.Subscriber('/scan1', LaserScan, lidar_callback)
pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
rospy.spin()