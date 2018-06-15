#!/usr/bin/env python


import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import numpy as np

import time

server = None

br = None
counter = 0
prev_time=0.0

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

        time_cur = rospy.get_time()
        print time_cur
        global prev_time
        print prev_time
        #publish clicked point
        if(time_cur> prev_time+4.0):
            msg=Pose()
            #msg.header.frame_id="map"
            #msg.header.stamp = rospy.Time.now()
            msg.position.x = float(feedback.mouse_point.x)
            msg.position.y = float(feedback.mouse_point.y)
            msg.position.z = float(feedback.mouse_point.z)
            pub.publish(msg)
            print "############pub#############"
            prev_time=time_cur
        

    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")

    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )

    
    server.applyChanges()




def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control


#####################################################################
# Marker Creation



def make6DofMarker( fixed, interaction_mode, position,num, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "simple_6dof"+str(num)
    

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        #int_marker.name += "_" + control_modes_dict[interaction_mode]
        #int_marker.description = "3D Control"
        
        #int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    
    server.insert(int_marker, processFeedback)
    




if __name__=="__main__":
    rospy.init_node("load_landing_sites")
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    pub = rospy.Publisher('/landing_sites_np/clicked_pose', Pose, queue_size=1)

    server = InteractiveMarkerServer("load_landing_sites")
    #load landing_sites numpy array and publish interative markers
    poses=np.load('airsim_test_py.npy')
   
    counter=0
    
    for pose in poses:
        if counter%1000==0:
            position = Point(pose.position.x, pose.position.y, pose.position.z)
            make6DofMarker( False, InteractiveMarkerControl.ROTATE_3D, position, counter, False)
        counter+=1
    

    server.applyChanges()

    rospy.spin()

