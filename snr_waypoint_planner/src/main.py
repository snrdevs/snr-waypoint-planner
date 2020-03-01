#!/usr/bin/python
 # -*- coding: utf-8 -*-
"""
SNR Waypoint Planner


"""


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

from snr_waypoint_planner.srv import service, serviceResponse
 
    

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
        self.service                                            = rospy.Service('service',service,self.service_cb)

        self.point_listener = rospy.Subscriber('/clicked_point', PointStamped, self.record_waypoints, queue_size=1)
    
        self.arb_path_file = None 
          
        # init x,y,z coordinates 
        self.x          =   [] 
        self.y          =   []
        self.yaw        =   []
        # new x and y values after interpolation 
        self.xnew       =   []
        self.ynew       =   []
        self.yaw        =   [] 
        self.odom_quat  =   None
        
        self.path2go                    = Path() 
        self.path2go.header.frame_id    =  'map'

       # Call main function 
        self.main() 

    def main(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown(): 
            self.waypoint_marker_publisher.publish(self.markerArray_wp)
            self.inp_marker_publisher.publish(self.markerArray_inp)

            self.path2go.header.stamp       =   rospy.Time.now() 
            self.path_publisher.publish(self.path2go)
            rate.sleep() 
            
    
    def record_waypoints(self, in_data):
            # take x and y and make z = 0
            self.cls() 
            self.x.append(in_data.point.x) 
            self.y.append(in_data.point.y)
            rospy.loginfo("Recording data \t | Total Number of Waypoints: {}".format(len(self.x)))
            
            # create waypoint markers 
            created_marker = self.create_marker('sphere',in_data.point.x,in_data.point.y,0,0,1,0)
            self.markerArray_wp.markers.append(created_marker)
            self.renumber_markers(self.markerArray_wp)

            
    def interpolate(self): 
        # create np arrays for manipulation 
        # a = [0, 2, 2, 0,0] # for testing! | creates rectangle
        # b = [0, 0, 2, 2,0] # for testing!  | creates rectangle 

        # pre interpolation for better smoothness
        a = self.x 
        b = self.y 
        x = np.array(a) 
        y = np.array(b)

        ctr = np.vstack((x,y)).T #stack x and y points 
        ctr[:,0] = x 
        ctr[:,1] = y

        tck,u = interpolate.splprep([x,y],k=1,s=0)  # linear interpolation k = 1 
        u=np.linspace(0,1,num=120,endpoint=True)    # with 120 points 
        out = interpolate.splev(u,tck)              # return generated points as array 
        
        # Start to B-Spline Interpolation
        x = np.array(out[0]) 
        y = np.array(out[1])

        ctr = np.vstack((x,y)).T                    # Stack x and y points 
        ctr[:,0] = x 
        ctr[:,1] = y

        tck,u = interpolate.splprep([x,y],k=3,s=0.0045)
        u=np.linspace(0,1,num=100,endpoint=True)
        out = interpolate.splev(u,tck)

        self.xnew = out[0]
        self.ynew = out[1]
        
        
    def find_yaw(self):
        #find yaw angles - #create yaw variable / create zero vector for memory allocation 
        self.yaw = np.zeros(len(self.xnew))

        t = 0 #counter for loop - start from 1 
        for t in range(len(self.xnew) -1 ):
            self.cls()
            self.yaw[t+1] = atan2(self.ynew[t+1]-self.ynew[t], self.xnew[t+1]-self.xnew[t])
            rospy.loginfo("Finding yaw angles")

        self.yaw[0] = self.yaw[1] # equalize second to first 
        self.yaw[-1] = self.yaw[-2] # equalize last to previous one

    def cls(self): 
        os.system('cls' if os.name == 'nt' else 'clear')

    def service_cb(self,user_req):
        if user_req.req == 1:
            self.cls()
            self.interpolate()
            self.find_yaw() 

            #show on rviz 
            if len(self.markerArray_inp.markers)<1:
                i = 0       
                for i in range(len(self.xnew)):
                    created_marker_inp = self.create_marker('arrow',self.xnew[i], self.ynew[i],1,0,0,self.yaw[i])
                    self.markerArray_inp.markers.append(created_marker_inp)
            else: 
                self.markerArray_inp.markers =[]        
            self.renumber_markers(self.markerArray_inp)

            
            # check write_to_file request
            if user_req.write_to_file == 1:
                self.write_to_file()
                self.cls() 
                rospy.loginfo("Path file generated to: " + self.arb_path_file)

            # check publish path request
            if user_req.publish_path == 1: 
                self.create_path() 
        return serviceResponse(1)

    def create_path(self):
        rospy.loginfo("Creating and publishing path...")

        for i in range(len(self.xnew)):
            # create pose 
            self.pose2write                 =   PoseStamped() 
            # set header 
            self.pose2write.header.frame_id =   'odom' #ok
            self.pose2write.header.seq      =   i
            self.pose2write.header.stamp    =   rospy.Time.now()
            # set position 
            self.pose2write.pose.position.x =   self.xnew[i]
            self.pose2write.pose.position.y =   self.ynew[i]
            # set orientation 
            quat_coord = tf.transformations.quaternion_from_euler(0,0,self.yaw[i])
            self.pose2write.pose.orientation.x = quat_coord[0]
            self.pose2write.pose.orientation.y = quat_coord[1]
            self.pose2write.pose.orientation.z = quat_coord[2]
            self.pose2write.pose.orientation.w = quat_coord[3]

            # create path and append poses 
            self.path2go.poses.append(self.pose2write)

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
        
    def create_marker(self, type,x,y,r,g,b,yaw):
        self.marker = Marker() 
        self.marker.header.frame_id="/map"
        if type == 'sphere': 
            self.marker.type = self.marker.SPHERE
        elif type == 'arrow':
            self.marker.type = self.marker.ARROW
        else: 
            self.marker.type = self.marker.SPHERE
            
        self.marker.action = self.marker.ADD
        
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02

        self.marker.color.a = 1.0
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b

        quat_ =  tf.transformations.quaternion_from_euler(0,0,yaw) 
        
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y 
        self.marker.pose.position.z = 0.0

        self.marker.pose.orientation.x = quat_[0] 
        self.marker.pose.orientation.y = quat_[1] 
        self.marker.pose.orientation.z = quat_[2] 
        self.marker.pose.orientation.w = quat_[3]
        
        return self.marker

    def renumber_markers(self,in_marker_arr):
        id = 0 
        for m_ in in_marker_arr.markers: 
            m_.id = id 
            id+=1 



if __name__ == "__main__":
    path_planner = WaypointPlanner()
    