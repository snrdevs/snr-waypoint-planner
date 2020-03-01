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


        self.waypoint_marker_pub_topic = 'waypoint_markers'
        self.inp_marker_pub_topic = 'inp_markers'
        self.path_topic       = 'snr_path'

        self.markerArray_wp = MarkerArray() 
        self.markerArray_inp = MarkerArray() 

        self.waypoint_marker_publisher                          = rospy.Publisher(self.waypoint_marker_pub_topic, MarkerArray,queue_size=10)
        self.inp_marker_publisher                               = rospy.Publisher(self.inp_marker_pub_topic, MarkerArray,queue_size=10)
        self.path_publisher                                     = rospy.Publisher(self.path_topic,Path,queue_size=10)
        self.show_interpolated_path_service                     = rospy.Service('path_publisher_trigger',path_publisher_trigger,self.show_interpolated_path)
        self.pp_trigger = True 

        self.resolution     = 0.001 #default 
        self.frequency      = 10 #hz 
        self.point_listener = rospy.Subscriber('/clicked_point', PointStamped, self.record_waypoints, queue_size=1)
        # self.odom_listener  = rospy.Subscriber('/odom', Odometry, self.odometry_holder, queue_size=1)
        
        self.starter        = 0
        self.arb_path_file = None 
        self.interp_method = 'quadratic' #['zero','linear', 'nearest', 'quadratic', 'cubic', 'previous', 'next']
          
        # init x,y,z coordinates 
        self.x          =   [] 
        self.y          =   []
        self.yaw        =   []
        # new x and y values after interpolation 
        self.xnew       =   []
        self.ynew       =   []
        self.yaw        =   [] 
        self.odom_quat  =   None
        
        self.path2go                = Path() 

       # Call main function 
        self.main() 

    def main(self):
        while not rospy.is_shutdown(): 
            self.waypoint_marker_publisher.publish(self.markerArray_wp)
            self.inp_marker_publisher.publish(self.markerArray_inp)
            # self.path_publisher.publish(self.path2go)
            rospy.Rate(30).sleep()
        pass
    
    
    def record_waypoints(self, in_data):
            # take x and y and make z = 0
            self.x.append(in_data.point.x) 
            self.y.append(in_data.point.y)

            # create waypoint markers 
            created_marker = self.create_marker(in_data.point.x,in_data.point.y,0,0,1)
            self.markerArray_wp.markers.append(created_marker)
            self.renumber_markers(self.markerArray_wp)
            print("#" * 30)
            print(self.x)
            print(self.y)
            print("#" * 30)
            
    def interpolate(self): 
        # create np arrays for manipulation 
        # a = [2, 2, 1, 0]
        # b = [0, 0.7, 0.7,0.7]
        a = self.x 
        b = self.y 
        
        #pre interpolation 
        x = np.array(a) 
        y = np.array(b)

        print(x.shape)
        print(y.shape)

        ctr = np.vstack((x,y)).T #stack x and y points 
        ctr[:,0] = x 
        ctr[:,1] = y

        tck,u = interpolate.splprep([x,y],k=1,s=0)
        u=np.linspace(0,1,num=100,endpoint=True)
        out = interpolate.splev(u,tck)

        xnew = out[0]
        ynew = out[1]

        x = np.array(xnew) 
        y = np.array(ynew)

        print(x.shape)
        print(y.shape)

        ctr = np.vstack((x,y)).T #stack x and y points 
        ctr[:,0] = x 
        ctr[:,1] = y

        tck,u = interpolate.splprep([x,y],k=3,s=0.005)
        u=np.linspace(0,1,num=100,endpoint=True)
        out = interpolate.splev(u,tck)

        xnew = out[0]
        ynew = out[1]
        
        self.xnew = out[0]
        self.ynew = out[1]
        
        
    def find_yaw(self):
        #find yaw angles - #create yaw variable / create zero vector for memory allocation 
        self.yaw = np.zeros(len(self.xnew))

        t = 0 #counter for loop - start from 1 
        for t in range(len(self.xnew) - 1):
            self.yaw[t+1] = atan2(self.ynew[t+1]-self.ynew[t], self.xnew[t+1]-self.xnew[t])

        #TODO: CHECK 
        # self.yaw[0] = self.yaw[1] # equalize second to first 
        # self.yaw[-1] = self.yaw[-2] # equalize last to previous one

    def show_interpolated_path(self,user_req):
        if user_req.req == 1:
            # os.system('cls' if os.name == 'nt' else 'clear')
            self.interpolate()
            
            # #show on rviz 
            # #re -create markerarray 
            
            if len(self.markerArray_inp.markers)<1:
                i = 0       
                for i in range(len(self.xnew)):
                    created_marker_inp = self.create_marker(self.xnew[i], self.ynew[i],1,0,0)
                    self.markerArray_inp.markers.append(created_marker_inp)
            else: 
                self.markerArray_inp.markers =[]
                    
            self.renumber_markers(self.markerArray_inp) 

        return path_publisher_triggerResponse(1)

            # self.write_to_file()
            # print("Path file generated to: " + self.arb_path_file)
            # print("-" * 100)
            
#             for i in range(len(self.xnew)):
# # TODO
#                 self.pose2write             = PoseStamped() 

#                 self.pose2write.header.frame_id = 'odom' #check 
#                 # self.pose2write.header.stamp    = rospy.Time.now()
        
#                 self.path2go.header.frame_id    = 'map'
#                 # self.path2go.header.stamp       =  rospy.Time.now() 

#                 self.pose2write.pose.position.x = self.xnew[i]
#                 self.pose2write.pose.position.y = self.xnew[i]

#                 self.arb_quat = tf.transformations.quaternion_from_euler(0,0,self.yaw[i])
#                 # self.pose2write.pose.orientation = tf.transformations.quaternion_from_euler(0,0,self.yaw[i])
#                 self.pose2write.pose.orientation.x = self.arb_quat[0]
#                 self.pose2write.pose.orientation.y = self.arb_quat[1]
#                 self.pose2write.pose.orientation.z = self.arb_quat[2]
#                 self.pose2write.pose.orientation.w = self.arb_quat[3]

                
#                 #pose2write created 
#                 self.path2go.poses.append(self.pose2write)

#                 print(self.path2go.poses)
#                 os.system('cls' if os.name == 'nt' else 'clear')
#                 print("publishing path..")
#                 # self.pp_trigger = True 
#             # print("odom_quat:" , self.odom_quat)

            
            # self.pose2write.pose.orientation.x = yaw_to_quat --> x 
            # self.pose2write.pose.orientation.y = yaw_to_quat --> y 
            # self.pose2write.pose.orientation.Z = yaw_to_quat --> z 
            



            # return path_publisher_triggerResponse(1)
    
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
        
    def create_marker(self, x,y,r,g,b):
        self.marker = Marker() 
        self.marker.header.frame_id="/map"
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02

        self.marker.color.a = 1.0
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b

        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y 
        self.marker.pose.position.z = 0.0

        return self.marker

    def renumber_markers(self,in_marker_arr):
        id = 0 
        for m_ in in_marker_arr.markers: 
            m_.id = id 
            id+=1 



if __name__ == "__main__":
    path_planner = WaypointPlanner()
    