#! /usr/bin/env python

import roslib; roslib.load_manifest('voice_command')
import rospy
import actionlib
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
import os

class populate_map():

    def __init__(self):
        rospy.init_node('initialize_map')
        rospy.Subscriber('voice_command/map', std_msgs.msg.String, self.update_dict_callback)

    def update_dict_callback(self, data):

        pub = rospy.Publisher('/youbot/visualization_marker_array', MarkerArray, queue_size=10)
        markerArray = MarkerArray()
        script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
        rel_path = "../config/location_dictionary.yaml"
        abs_file_path = os.path.join(script_dir, rel_path)
        locations = None
        with open(abs_file_path,'r') as myfile:
            locations = myfile.readlines()

        count = 0
        for location in locations:
            name_and_loc = location.split('{')
            # handle name
            name_orig = name_and_loc[0]
            name = name_orig.strip().replace(':','').replace('_',' ')
            # handle location
            full_loc_orig = name_and_loc[1].replace('}','')
            all_loc_parts = full_loc_orig.split(',')
            pos_x = float(all_loc_parts[0].split(':')[1].strip())
            pos_y = float(all_loc_parts[1].split(':')[1].strip())
            pos_z = float(all_loc_parts[2].split(':')[1].strip())
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.ns = 'map_labels'
            marker.id = count
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.position.x = pos_x
            marker.pose.position.y = pos_y
            marker.pose.position.z = pos_z
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.text = name
            marker.color.b = 0.8
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            markerArray.markers.append(marker)
            count = count + 1

        pub.publish(markerArray)

if __name__ == '__main__':
    try:
        populate_map()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished updating map")
