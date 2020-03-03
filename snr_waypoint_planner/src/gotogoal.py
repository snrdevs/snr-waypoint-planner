#!/usr/bin/python
 # -*- coding: utf-8 -*-

# ROS Packages 
import rospy 
import rospkg
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, Twist
import tf 

import numpy as np 
from math import atan2, cos, sin


class GoToGoal(object):
    def __init__(self):
        self.path_sub = rospy.Subscriber('/snr_path', Path, self.listen_path, queue_size=10)          
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.listen_odom, queue_size=10)          

        self.cur_x      = None 
        self.cur_y      = None 
        self.cur_yaw    = None 

        self.cur_quat_x = None 
        self.cur_quat_y = None 
        self.cur_quat_z = None 
        self.cur_quat_w = None 
        
        self.desired_vel = Twist() 
        self.desired_vel.linear.x = 0.1

        self.rate = rospy.Rate(30)
        self.path = Path() 

    def listen_path(self, path):
        self.path = path 

    def listen_odom(self, odom_data):
        pose = odom_data.pose.pose 
        #position 
        self.cur_x = pose.position.x
        self.cur_y = pose.position.x
        #orientation
        self.cur_quat_x = pose.orientation.x
        self.cur_quat_y = pose.orientation.y
        self.cur_quat_z = pose.orientation.z
        self.cur_quat_w = pose.orientation.w

        quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        self.cur_yaw = rpy[2]

        rospy.loginfo("x:{} y:{} yaw:{}".format(self.cur_x,self.cur_y,self.cur_yaw))
        pass

    def gotogoal(self):

        """for diff drive mobile robots : 
        omega = R/L * (V_r - V_l) --> Angular Velocity, angular.z 
        V = /L * (V_r + V_l) 
        _______
        _______
        x_dot = V * cos(phi) --> phi : yaw
        y_dot = V * sin(phi) 
        phi_dot = omega

        """

        i = 0 
        kp = 0.5

        # for pose in self.path.poses:
        #     x_goal[i] = self.path.poses[i].pose.position.x
        #     y_goal[i] = self.path.poses[i].pose.position.y
             
        # quaternion[i]  =  tf.transformations.euler_from_quaternion()
            
        # x_goal = path.poses[i].pose.position.x 
        # y_goal = path.poses[i].pose.position.y


        # quaternion = (path_quat_x,path_quat_y,path_quat_z,path_quat_w) 
        # rpy = tf.transformations.euler_from_quaternion(quaternion)
        # yaw_goal = rpy[2]

        # goal_vector = [x_goal, y_goal, yaw_goal] 
        # return goal_vector
        pub_topic = 'cmd_vel'
        pub = rospy.Publisher(pub_topic, Twist, queue_size=10)


        while True:            
            x_goal = self.path.poses[i].pose.position.x 
            y_goal = self.path.poses[i].pose.position.y 
            
            quat_x_goal = self.path.poses[i].pose.orientation.x
            quat_y_goal = self.path.poses[i].pose.orientation.y
            quat_z_goal = self.path.poses[i].pose.orientation.z
            quat_w_goal = self.path.poses[i].pose.orientation.w

            quaternion = (quat_x_goal,quat_y_goal,quat_z_goal,quat_w_goal) 
            quaternion = tf.transformations.euler_from_quaternion(quaternion) 
            
            yaw_goal = quaternion[2]
            

            # if (self.cur_x in range(x_goal-tolerance,y_goal+tolerance, 0.001)):

            # if (self.cur_x == x_goal and self.cur_y == y_goal and self.cur_yaw == yaw_goal):
                # i += 1 
            # else: 
            error = yaw_goal - self.cur_yaw 
            self.desired_vel.angular.z = error * kp 
            pub.publish(self.desired_vel)

            if x_goal == self.cur_x and y_goal == self.cur_y and yaw_goal == self.cur_yaw: 
                break 

            self.rate.sleep()             

if __name__ == "__main__":
    rospy.init_node('gotogoal_node')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() : 
        gotogoal = GoToGoal() 



    
    