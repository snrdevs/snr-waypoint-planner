#!/usr/bin/python
 # -*- coding: utf-8 -*-
import os 

import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

from scipy import interpolate
from math import atan2, cos, sin

# ROS Packages 
import rospy, rospkg, tf
rospack = rospkg.RosPack()

from nav_msgs.msg import Path, Odometry 
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
#rviz libs
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker, MarkerArray

from snr_waypoint_planner.srv import path_publisher_trigger, path_publisher_triggerResponse
 
    

class WaypointPlanner:
    
    def __init__(self):

        rospy.init_node('waypoint_planner',anonymous=False)

        self.marker_pub_topic = 'waypoint_markers'
        self.path_topic       = 'snr_path'

        self.markerArray = MarkerArray() 

        self.marker_publisher                       = rospy.Publisher(self.marker_pub_topic, MarkerArray,queue_size=10)
        self.path_publisher                         = rospy.Publisher(self.path_topic,Path,queue_size=10)
        self.path_publish_trigger_service           = rospy.Service('path_publisher_trigger',path_publisher_trigger,self.path_publisher_trigger_cb)
        self.pp_trigger = True 

        self.resolution     = 0.01 #default 
        self.frequency      = 10 #hz 
        self.point_listener = rospy.Subscriber('/clicked_point', PointStamped, self.record, queue_size=1)
        self.odom_listener  = rospy.Subscriber('/odom', Odometry, self.odometry_holder, queue_size=1)
        
        self.starter        = 0
        self.arb_path_file = None 
        self.interp_method = ['zero','linear', 'nearest', 'quadratic', 'cubic', 'previous', 'next']
          
        # init x,y,z coordinates 
        self.x          =   [] 
        self.y          =   []
        self.z          =   []
        self.yaw        =   []
        # new x and y values after interpolation 
        self.xnew       =   []
        self.ynew       =   []
        self.yaw        =   [] 
        self.odom_quat  =   None
        
        # create odom variables 
        self.odom_header            = None 
        self.odom_data_flow_barrier = False
        self.path2go                = Path() 

        # Figure variables 
        self.ax         =   None      # axis 
        self.fig        =   plt.figure(figsize=(10,20))# figure
        self.ax         =   self.fig.add_subplot(111)

        # Call main function 
        self.main() 

    def main(self):
        while not rospy.is_shutdown(): 
            self.marker_publisher.publish(self.markerArray)
            self.path_publisher.publish(self.path2go)
            rospy.Rate(30).sleep()
        pass
    
    
    def record(self, in_data):
        if self.pp_trigger == True: 
            # take x and y and make z = 0
            x_ = in_data.point.x 
            y_ = in_data.point.y 
            z_ = 0.0 
            
            self.x.append(x_) 
            self.y.append(y_)
            self.z.append(z_)
            
            # create markers 
            created_marker = self.create_marker(x_,y_)
            
            #calculate yaw within the created markers 
            if len(self.x) >= 4 and len(self.y) >=4: #interpolation gives error below 4 
                self.find_yaw()
                for t = 0 in range(len(self.xnew)): 
                    created_marker = self.create_marker(self.xnew[t],self.ynew[t])
                    self.markerArray.markers.append(created_marker)
                self.renumber_markers() 

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
        self.f = interpolate.interp1d(self.x, self.y, kind=self.interp_method[4])

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

        #TODO: CHECK 
        # self.yaw[0] = self.yaw[1] # equalize second to first 
        # self.yaw[-1] = self.yaw[-2] # equalize last to previous one 

    def path_publisher_trigger_cb(self,user_req):
        if (user_req.req == 1):
            self.pp_trigger = True
            return path_publisher_triggerResponse(1) 
        else: 
            self.pp_trigger   = False
            os.system('cls' if os.name == 'nt' else 'clear')
            print("Recording stopped. Generating path file...")
            rospy.sleep(1) # sleep :) 
            self.write_to_file()
            print("Path file generated to: " + self.arb_path_file)
            print("-" * 100)
            # show_plot = input("Do you want to see the graph of generated path? (Y/N): ")
            # if show_plot == 'Y' or show_plot == 'y': 
            # self.show_path() 

            # clear path2 go and assign the vars 

            
            for i in range(len(self.xnew)):
# TODO
                self.pose2write             = PoseStamped() 

                self.pose2write.header.frame_id = 'odom' #check 
                # self.pose2write.header.stamp    = rospy.Time.now()
        
                self.path2go.header.frame_id    = 'map'
                # self.path2go.header.stamp       =  rospy.Time.now() 

                self.pose2write.pose.position.x = self.xnew[i]
                self.pose2write.pose.position.y = self.xnew[i]

                self.arb_quat = tf.transformations.quaternion_from_euler(0,0,self.yaw[i])
                # self.pose2write.pose.orientation = tf.transformations.quaternion_from_euler(0,0,self.yaw[i])
                self.pose2write.pose.orientation.x = self.arb_quat[0]
                self.pose2write.pose.orientation.y = self.arb_quat[1]
                self.pose2write.pose.orientation.z = self.arb_quat[2]
                self.pose2write.pose.orientation.w = self.arb_quat[3]

                
                #pose2write created 
                self.path2go.poses.append(self.pose2write)

                print(self.path2go.poses)
                os.system('cls' if os.name == 'nt' else 'clear')
                print("publishing path..")
                # self.pp_trigger = True 
            # print("odom_quat:" , self.odom_quat)

            
            # self.pose2write.pose.orientation.x = yaw_to_quat --> x 
            # self.pose2write.pose.orientation.y = yaw_to_quat --> y 
            # self.pose2write.pose.orientation.Z = yaw_to_quat --> z 
            



            return path_publisher_triggerResponse(0)
    
    def write_to_file(self):
        #clear file contents and write  
        package_path = rospack.get_path('snr_waypoint_planner')
        
        if not os.path.exists(package_path + '/planned_path/'):
		    os.makedirs(package_path + '/planned_path/')    
        planned_path_txt_dir = package_path + '/planned_path/'
        
        i = 0
        while True:
		    dname = planned_path_txt_dir+'%03d'%i
		    if os.path.exists(dname):
		        i += 1
		    else:
		        os.makedirs(dname)
		        break
        filename = dname + '/planned_path.txt'
        file = open(filename, 'w')
        self.arb_path_file = filename 
        
        i=0
        if len(self.xnew) == len(self.ynew):    
            for i in range(len(self.xnew)):
                self.coord2write = str(self.xnew[i]) + '\t' + str(self.ynew[i]) + '\t' + str(self.yaw[i]) + '\n'
                file.write(self.coord2write)
            file.close()
                
    def show_path(self):
        #show matplotlib plot
        # TODO: CHECK !! 
        # self.ax.clear()
        plt.plot(self.x, self.y, 'o', self.xnew, self.ynew, '-')
        plt.legend(['waypoints','interpolated_path'])
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
    path_planner = WaypointPlanner()
    