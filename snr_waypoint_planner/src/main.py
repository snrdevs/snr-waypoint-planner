#!/usr/bin/python
 # -*- coding: utf-8 -*-

import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

from scipy import interpolate
from math import atan2, cos, sin

# ROS Packages 
import rospy, rospkg
from nav_msgs.msg import Path, Odometry 
from geometry_msgs.msg import PointStamped, PoseStamped
#rviz libs
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray

from snr_waypoint_planner.srv import path_publisher_trigger
 
    

class WaypointPlanner:
    
    def __init__(self):

        rospy.init_node('waypoint_planner',anonymous=False)

        self.marker_pub_topic = 'waypoint_markers'
        self.path_topic       = 'hsn_path'
        self.service_topic    = 'time2publish_path'

        self.path2go = Path() 
        self.markerArray = self.markerArray() 


        self.marker_publisher                       = rospy.Publisher(self.marker_pub_topic, self.markerArray,queue_size=10)
        self.path_publisher                         = rospy.Publisher(self.path_topic,Path,queue_size=10)
        self.path_publish_trigger_service           = rospy.Service('path_publisher_trigger',path_publisher_trigger,self.path_publisher_trigger)

        self.resolution     = 0.01 #default 
        self.frequency      = 10 #hz 
        self.point_listener = rospy.Subscriber('/clicked_point', PointStamped, self.record, queue_size=1)
        self.odom_listener  = rospy.Subscriber('/odom', Odometry, self.odometry_holder, queue_size=1)
        self.starter        = 0
          
        # init x,y,z coordinates 
        self.x          =   [] 
        self.y          =   []
        self.z          =   []
        self.yaw        =   []
        # new x and y values after interpolation 
        self.xnew       =   []
        self.ynew       =   []
        self.yaw        =   [] 
        
        # create odom variables 
        self.odom_header  = None 
        self.odom_data_flow_barrier = False
        self.pose2write = PoseStamped() 

        # Figure variables 
        self.ax         = None      # axis 
        self.fig = plt.figure(figsize=(10,20))# figure
        self.ax = self.fig.add_subplot(111)

        # Call main function 
        self.main() 

    def main(self):
        while not rospy.is_shutdown(): 
            self.marker_publisher.publish(self.markerArray)
            if len(self.x) >= 4 and len(self.y) >=4: 
                self.path_publisher.publish(self.path2go)

            rospy.Rate(10).sleep()
        pass
    
    
    def record(self, in_data):
            # take x and y and make z = 0
            x_ = in_data.point.x 
            y_ = in_data.point.y 
            z_ = 0.0 
            
            self.x.append(x_) 
            self.y.append(y_)
            self.z.append(z_)
            
            # create markers 
            created_marker = self.create_marker(x_,y_)
            self.markerArray.markers.append(created_marker)
            self.renumber_markers() 

            #calculate yaw within the created markers 
            if len(self.x) >= 4 and len(self.y) >=4: #interpolation gives error below 4 
                self.find_yaw()

                print("x_values: " , self.xnew)
                print("y_values: " , self.ynew)
                print("yaw_values: " , self.yaw)

                print("lengths: " , len(self.xnew), len(self.ynew), len(self.yaw))

            # self.show_path() 

    def odometry_holder(self, realtime_odom_data):
        if (self.odom_data_flow_barrier == False): 
            self.odom_header = realtime_odom_data.header 
            self.pose2write.header = realtime_odom_data.header 

            self.odom_data_flow_barrier = True # convert to true so it will not work always 
            


    def find_yaw(self):
        # select interpolation method 
        """Specifies the kind of interpolation as a string (‘linear’, ‘nearest’, ‘zero’, ‘slinear’, ‘quadratic’, ‘cubic’, ‘previous’, ‘next’, where ‘zero’, ‘slinear’, ‘quadratic’ and ‘cubic’ refer to a spline interpolation of zeroth, first, second or third order; ‘previous’ and ‘next’ simply return the previous or next value of the point) or as an integer specifying the order of the spline interpolator to use. Default is ‘linear’."""
        self.f = interpolate.interp1d(self.x, self.y, kind='cubic')

        #create new array for x and y values and evalute with f 
        self.xnew = np.arange(self.x[0], self.x[-1], self.resolution)
        self.ynew = self.f(self.xnew) 

        #find yaw angles - #create yaw variable / create zero vector for memory allocation 
        self.yaw = np.zeros(len(self.xnew))

        t = 0 #counter for loop - start from 1 
        for t in range(len(self.xnew) - 1):
            # TODO : CHECK 
            # self.x_forward  = self.xnew[t+1]
            # self.x_backward = self.xnew[t-1]
            # self.y_forward  = self.ynew[t+1]
            # self.y_backward = self.ynew[t-1]
            if (t == 0): 
                self.yaw[0] = 0 # give some initial value for zeroth index 
            self.yaw[t+1] = atan2(self.ynew[t+1]-self.ynew[t], self.xnew[t+1]-self.xnew[t])

        # self.yaw[0] = self.yaw[1] # equalize second to first 
        # self.yaw[-1] = self.yaw[-2] # equalize last to previous one 

    def path_publisher_trigger(self):
        pass
    
    def write_to_file(self):
        #clear file contents and write  
        file = open('planned_path.txt', 'w')
        #write to file 

        i=0
        if len(self.xnew) == len(self.ynew):    
            for i in range(len(self.xnew)):
                self.coord2write = str(self.xnew[i]) + '\t' + str(self.ynew[i]) + '\t' + str(self.yaw[i]) + '\n'
                file.write(self.coord2write)
            file.close()
                    
    def show_path(self):
        #show matplotlib plot
        # TODO: CHECK !! 
        self.ax.clear()
        self.ax.plot(self.x, self.y, 'o', self.xnew, self.ynew, '-')
        self.ax.legend(['waypoints','interpolated_path'])
        plt.show() 

    def clear_all(self): 
        self.x = [] 
        self.y = [] 
        self.z = []
        self.yaw = [] 
        
    def create_marker(self, x,y):
        self.marker = Marker() 
        self.marker.header.frame_id="/map"
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0

        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y 
        self.marker.pose.position.z = 0.0

        return self.marker

    def renumber_markers(self):
        id = 0 
        for m_ in self.markerArray.markers: 
            m_.id = id 
            id+=1 



if __name__ == "__main__":
    
    #init class 
    path_planner = WaypointPlanner()
    
    # show it on rviz as nav_msgs.msg->path 
