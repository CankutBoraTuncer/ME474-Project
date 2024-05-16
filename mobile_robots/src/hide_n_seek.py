#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
from geometry_msgs.msg import PoseStamped
import time

class HideNSeek():

    def __init__(self):
        self.target_list = [ [3.4, 0.6] , [3.2, 0.6] , [3.0, 0.6] , [2.8, 0.8] , [2.6, 1.0] , [2.4, 1.0] , 
                            [2.1, 1.0] , [2.0, 1.0] , [1.8, 1.0] , [1.6, 0.8] , [1.6, 0.6] , [1.4, 0.4] , 
                            [1.2, 0.2] , [1.0, 0.2] , [0.8, 0.2] , [0.6, 0.2] , [0.6, 0.4] , [0.6, 0.6] , 
                            [0.6, 0.8] , [0.6, 1.0] , [0.8, 1.2] , [1.0, 1.2] , [1.2, 1.2] , [1.4, 1.3] , 
                            [1.4, 1.6] , [1.4, 1.8] , [1.4, 2.0] , [1.2, 2.2] , [1.0, 2.4] , [0.8, 2.8] , 
                            [0.3, 2.8] , [0.2, 3.0] , [0.2, 3.2] , [0.2, 3.4] , [0.2, 3.2] , [0.2, 3.0] , 
                            [0.2, 2.8] , [0.2, 2.6] , [0.2, 2.4] , [0.4, 2.2] , [0.6, 2.4] , [0.6, 2.6] , 
                            [0.8, 2.8] , [1.0, 3.0] , [1.2, 3.0] , [1.4, 3.0] , [1.6, 3.0] , [1.8, 3.0] , 
                            [2.0, 3.0] , [2.2, 3.0] , [2.4, 3.0] , [2.6, 3.0] , [2.8, 3.0] , [3.0, 3.0] , 
                            [3.2, 2.8] , [3.3, 2.6] , [3.6, 2.6] , [3.8, 2.8] , [3.8, 3.0] , [3.8, 3.2] , [3.8, 3.4] ]
        self.goal_tolerance = 0.2
        self.target_index = 0
        self.initCount = 1
        self.stuck_count = 0
        self.prev_pose = np.array([0, 0])


        rospy.init_node('line_fitter', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.get_set_position)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Rate(30)

        time.sleep(0.1)

    def get_set_position(self, msg):

        if self.target_index < len(self.target_list):

            pose = msg.pose.pose
            robot_x = pose.position.x
            robot_y = pose.position.y
            
            target = self.target_list[self.target_index]
            dist = np.linalg.norm([target[0] - robot_x, target[1] - robot_y])

            if self.initCount % 10 != 0:
                print(f"Position: x: {robot_x}, y: {robot_y}, target_idx: {self.target_index}")
                self.initCount +=1
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map" 

                pos_x = self.target_list[self.target_index][0]
                pos_y = self.target_list[self.target_index][1]

                goal.pose.position.x = pos_x
                goal.pose.position.y = pos_y
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 1

                self.goal_publisher.publish(goal)

            if dist <= self.goal_tolerance:
                self.stuck_count = 0
                print(f"Position: x: {robot_x}, y: {robot_y}, target_idx: {self.target_index}")
                self.target_index += 1

                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map" 

                pos_x = self.target_list[self.target_index][0]
                pos_y = self.target_list[self.target_index][1]

                goal.pose.position.x = pos_x
                goal.pose.position.y = pos_y
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 1

                self.goal_publisher.publish(goal)

            elif np.linalg.norm(self.prev_pose - np.array([robot_x, robot_y])) <= 0.001:
                self.stuck_count += 0.8
            
            if self.stuck_count > 100:
                print("NEXT")
                self.stuck_count = 0
                print(f"Position: x: {robot_x}, y: {robot_y}, target_idx: {self.target_index}")
                self.target_index += 1

                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map" 

                pos_x = self.target_list[self.target_index][0]
                pos_y = self.target_list[self.target_index][1]

                goal.pose.position.x = pos_x
                goal.pose.position.y = pos_y
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 1

                self.goal_publisher.publish(goal)

            self.prev_pose = np.array([robot_x, robot_y])

            

if __name__ == '__main__':
    line_fitter = HideNSeek()
    rospy.spin()