#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapFusion:
    def __init__(self):
        rospy.init_node('map_filter', anonymous=True)

        self.premade_map = None
        self.gmapping_map = None
        self.fused_map = None
        
        # Kalman filter parameters
        self.Q = 0.01  # Process noise covariance
        self.R = 0.1   # Measurement noise covariance
        self.P = np.ones((352, 352))  # Estimate error covariance (initialized to ones)
        self.rate = rospy.Rate(10)
    
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.gmapping_callback)
        self.premade_map_sub = rospy.Subscriber('/premade_map', OccupancyGrid, self.premade_map_callback)
        self.fused_map_pub = rospy.Publisher('/fused_map', OccupancyGrid, queue_size=10)
        
    def premade_map_callback(self, msg):
        self.premade_map = msg
        self.fuse_maps()

    def gmapping_callback(self, msg):
        self.gmapping_map = msg
        self.fuse_maps()

    def fuse_maps(self):
        if self.premade_map is not None and self.gmapping_map is not None:
            print("MAPS ARE RECEIVED!!")
            # Convert OccupancyGrid to numpy arrays
            premade_map_data = np.array(self.premade_map.data).reshape((self.premade_map.info.height, self.premade_map.info.width))
            gmapping_map_data = np.array(self.gmapping_map.data).reshape((self.gmapping_map.info.height, self.gmapping_map.info.width))

            # Apply Kalman filter to fuse premade_map_data and gmapping_map_data
            fused_map_data = self.kalman_filter(premade_map_data, gmapping_map_data)

            # Convert numpy array back to OccupancyGrid
            fused_map_msg = OccupancyGrid()
            fused_map_msg.header.stamp = rospy.Time.now()
            fused_map_msg.header.frame_id = "fused_map"
            fused_map_msg.info = self.gmapping_map.info
            fused_map_msg.data = fused_map_data.flatten().tolist()

            print("MAP ARE FUSED!!")
            self.fused_map_pub.publish(fused_map_msg)
            #self.rate.sleep()

    def kalman_filter(self, premade_map, gmapping_map):
        fused_map = np.zeros_like(premade_map)

        for i in range(premade_map.shape[0]):
            for j in range(premade_map.shape[1]):
                # Predict step
                x_pred = premade_map[i, j]
                P_pred = self.P[i, j] + self.Q

                # Update step
                z = gmapping_map[i, j]
                y = z - x_pred  # Measurement residual
                S = P_pred + self.R  # Residual covariance
                K = P_pred / S  # Kalman gain

                x_fused = x_pred + K * y
                self.P[i, j] = (1 - K) * P_pred

                fused_map[i, j] = x_fused

        return fused_map

if __name__ == '__main__':
    mf = MapFusion()
    rospy.spin()
