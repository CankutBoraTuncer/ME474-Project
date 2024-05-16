#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
import numpy as np

class robot(object):
    def __init__(self):
        self.MAX_TRANS_SPEED = 0.22 / 2
        self.MAX_ANG_SPEED   = 2.84
        self.WHEEL_RADIUS    = 0.033
        self.SPEED_REDUCTION_CONSTANT = 0.5
        self.WHEEL_L = 0.08
        self.MAX_WHEEL_SPEED = (self.MAX_TRANS_SPEED / self.WHEEL_RADIUS ) * self.SPEED_REDUCTION_CONSTANT
        
        self.list_encoder = []
        self.list_encoder_csv = []
        self.list_lidar = []
        self.header =['time','right','left']
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.pos = [0, 0]
        self.angle = np.pi
        self.pose = np.transpose([*self.pos, self.angle])

        J_f = np.array([[-np.sin(-np.pi/2 + 0), np.cos(-np.pi/2 + 0), self.WHEEL_L*np.cos(0)],
                        [-np.sin(np.pi/2 + np.pi), np.cos(np.pi/2 + np.pi), self.WHEEL_L*np.cos(np.pi)]])
        J2 = np.array([[self.WHEEL_RADIUS, 0], [0, self.WHEEL_RADIUS]])
        C_f = np.array([[np.cos(-np.pi/2 + 0), np.sin(-np.pi/2 + 0), self.WHEEL_L*np.sin(0)],
                        [np.cos(np.pi/2 + np.pi), np.sin(np.pi/2 + np.pi), self.WHEEL_L*np.sin(np.pi)]])
        self.A = np.vstack((J_f, C_f))
        self.B = np.vstack((J2, np.array([[0, 0], [0, 0]])))

        rospy.init_node('turtlebot_motion', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("/joint_states", JointState, self.chatter_callback)

        rospy.Subscriber("/scan", LaserScan, self.chatter_callback2)
        
        time.sleep(3)

    def set_left_wheel_velocity(self,velocity_in_rad_per_sec):
        velocity_message = Twist()
        self.left_wheel_velocity = velocity_in_rad_per_sec
        velocity_message.linear.x = self.limit_linear_speed(self.WHEEL_RADIUS*(self.left_wheel_velocity+self.right_wheel_velocity)/2)
        velocity_message.angular.z = self.limit_angular_speed(self.WHEEL_RADIUS*(self.right_wheel_velocity-self.left_wheel_velocity)/0.160)
        self.velocity_publisher.publish(velocity_message)

    def set_right_wheel_velocity(self,velocity_in_rad_per_sec):
        velocity_message = Twist()
        self.right_wheel_velocity = velocity_in_rad_per_sec
        velocity_message.linear.x = self.limit_linear_speed(self.WHEEL_RADIUS*(self.left_wheel_velocity+self.right_wheel_velocity)/2)
        velocity_message.angular.z = self.limit_angular_speed(self.WHEEL_RADIUS*(self.right_wheel_velocity-self.left_wheel_velocity)/0.160)
        self.velocity_publisher.publish(velocity_message)

    def publish_wheel_velocity(self):
        velocity_message = Twist()
        velocity_message.linear.x = self.limit_linear_speed(self.WHEEL_RADIUS*(self.left_wheel_velocity+self.right_wheel_velocity)/2)
        velocity_message.angular.z = self.limit_angular_speed(self.WHEEL_RADIUS*(self.right_wheel_velocity-self.left_wheel_velocity)/0.160)
        loop_rate = rospy.Rate(30)
        #print("Velocity: ", [velocity_message.linear.x, velocity_message.angular.z])
        #print([rospy.Time.now().to_sec(), self.right_wheel_velocity, self.left_wheel_velocity])
        self.list_encoder_csv.append([rospy.Time.now().to_sec(), self.right_wheel_velocity, self.left_wheel_velocity])
        
        tic = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - tic < 1):
            self.velocity_publisher.publish(velocity_message)
            loop_rate.sleep()      
     

    def limit_linear_speed(self, speed):
        if(speed >= self.MAX_TRANS_SPEED):
            speed = self.MAX_TRANS_SPEED
        elif(speed < -self.MAX_TRANS_SPEED):
            speed = -self.MAX_TRANS_SPEED
        return speed

    def limit_angular_speed(self, speed):
        if(speed >= self.MAX_ANG_SPEED):
            speed = self.MAX_ANG_SPEED
        elif(speed < -self.MAX_ANG_SPEED):
            speed = -self.MAX_ANG_SPEED
        return speed
    
    def chatter_callback(self,message):
        time1 = "%s" % rospy.get_time()
        fileHandle = open('mobile1.txt', 'a')
        time2 = "%f" % rospy.get_time()
        fileHandle.write(time1+"\t"+str(message.position[0])+"\t"+str(message.position[1])+'\n')
        en_data = [time2,message.position[0],message.position[1]]
        print(en_data)
        self.list_encoder.append(en_data)
        fileHandle.close()

    def chatter_callback2(self,message):
        self.list_lidar.append(message.ranges)

    def get_encoder_values(self):
        with open('encoder_path_4.csv', 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            # write the header
            writer.writerow(self.header)
            # write multiple rows
            writer.writerows(self.list_encoder)

    def get_lidar_values(self):
        with open('lidar.csv', 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            # write multiple rows
            writer.writerows(self.list_lidar)

    def R(self, angle):
        return np.array([[np.cos(angle), -np.sin(angle), 0],
                        [np.sin(angle), np.cos(angle), 0],
                        [0, 0, 1]])
    
    def inverse_kinematics(self, dx):
        return np.dot(np.dot(np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(self.B), self.B)) , np.transpose(self.B)) , self.A) , self.R(self.pose[2])) , dx)

    def move_bot_open_loop(self, next_pose):
        for npi in next_pose:
            dx = self.pose - npi
            [self.right_wheel_velocity, self.left_wheel_velocity] = self.inverse_kinematics(dx)
            #print("Wheel Vel: ", [self.right_wheel_velocity, self.left_wheel_velocity])
            self.pose = npi
            #print("Pose: ", self.pose)
            self.publish_wheel_velocity()
            
    def move_bot_on_circle(self, r, turn_angle, step_count):
        positions = []
        x = self.pose[0]
        y = self.pose[1]
        phi = self.pose[2]

        turn_angle *= -1
        step_angle = turn_angle / step_count

        base_angle = phi + turn_angle
        origin_x = x + r*np.cos(base_angle)
        origin_y = y + r*np.sin(base_angle)
        
        for i in range(0, step_count):
            phi += step_angle
            x = origin_x - np.cos( base_angle + step_angle * (i+1) ) * r
            y = origin_y + np.sin( base_angle + step_angle * (i+1) ) * r
  
            positions.append(np.array([x, y, phi]))

        return positions

if __name__ == '__main__':
    try:
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_model_srv = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        reset_model_srv()
        robot = robot()

        # USE THE SPACE BELOW FOR YOUR CODE
        # !!!!!!!!!!!! DON'T CHANGE THE PARTS THAT ARE ALREADY IN THE FILE  !!!!!!!!!!!!
        # You might need to use time.sleep({a time value in seconds}) for delay.
        
        """
        ## PATH 1 
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/4, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/4, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0, 0]]) # STOP
        """

        """
        ## PATH 2
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/4, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/4, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, -0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, -0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, -0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, -0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])    
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])   
        robot.move_bot_open_loop([robot.pose + [0, 0, 0]]) # STOP
        """

        
        ## PATH 3
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/4, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/4, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0, 0]]) # STOP
        

        """
        ## PATH 4
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/4, step_count = 3))
        time.sleep(1)
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/4, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = -np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0.1, 0]])
        robot.move_bot_open_loop(robot.move_bot_on_circle(r = 0.4, turn_angle = np.pi/2, step_count = 3))
        robot.move_bot_open_loop([robot.pose + [-0.1, 0, 0]])
        robot.move_bot_open_loop([robot.pose + [0, 0, 0]]) # STOP
        """

        print("Stop!")
        robot.get_encoder_values()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
