#!/usr/bin/python
 # -*- coding: utf-8 -*-

import numpy as np 
import matplotlib.pyplot as plt 
from scipy import interpolate
from math import atan2, cos, sin

# ROS Packages 
import rospy 
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
#rviz libs
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class WaypointPlanner:
    def __init__(self):
        self.resolution = 0.01 #default 
        self.frequency = 10 #hz 
        self.point_listener = rospy.Subscriber('/clicked_point', PointStamped, self.record, queue_size=1)
        self.rviz_visual_waypoints  = rospy.Publisher('/waypoints', PointStamped,queue_size=10)
        self.starter = 0  
        #init x,y,z coordinates 
        self.x = [] 
        self.y = []
        self.z = []

        self.rviz_points_data = PointStamped() 
        
    def record(self, in_data):
        if self.starter ==1 : 
            # take x and y and make z = 0
            self.x.append(in_data.point.x) 
            self.y.append(in_data.point.y)
            self.z.append(0)

            #create interactive marker on rviz 
            self.rviz_visual_waypoints.publish()
    def stop_recording(self):
        self.starter = 0 

    def find_path(self):
        pass

        #listen /clicked_point topic
        # insert points into an array 
        # make z = 0 
        # generate the path 
        # show the generated path on the screen 
        # show it on rviz as nav_msgs.msg->path 

if __name__ == "__main__":
    #init class 
    waypoint_listener = WaypointPlanner()
    start = input("Please enter 1 to start recording: ")
    if start == 1: 
        waypoint_listener.starter = 1 
    else: 
        waypoint_listener.stop_recording() 


    rospy.spin() 
    
