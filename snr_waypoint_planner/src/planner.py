#!/usr/bin/python
 # -*- coding: utf-8 -*-

import numpy as np 
import matplotlib.pyplot as plt 
from scipy import interpolate
import math 

# ROS Packages 
import rospy 
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
#rviz libs
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


rospy.init_node('waypoint_listener',anonymous=False)
marker_pub_topic = 'waypoint_markers'
publisher = rospy.Publisher(marker_pub_topic, MarkerArray,queue_size=10)

markerArray = MarkerArray() 

class MarkerArrayCreator: 
    def __init__(self):
        self.count  = 0 
        self.MARKERS_MAX = 50 
    
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
        self.marker.pose.position.x = math.cos(self.count / 30)
        self.marker.pose.position.y = math.cos(self.count / 40 ) 
        self.marker.pose.position.z = 0.0
    
    def renumber_markers(self):
        id = 0 
        for m_ in markerArray.markers: 
            m_.id = id 
            id+=1 
    
mm_ = MarkerArrayCreator() 
mm_.count = 0
    
# Publish the MarkerArray
while not rospy.is_shutdown():
    mm_.create_marker(0,0)
    
    # if(mm_.count > mm_.MARKERS_MAX): 
    #     markerArray.markers.pop(0)
    
    markerArray.markers.append(mm_.marker)

    mm_.renumber_markers() 
    publisher.publish(markerArray)

    mm_.count+=1 
    rospy.sleep(0.01)
        


    # create_marker_array(math.cos(count/30),math.cos(count/40))
    # if (count > MARKER_MAX): 
    #     markerArray.markers.pop(0) 
    # publisher.publish(markerArray)
 