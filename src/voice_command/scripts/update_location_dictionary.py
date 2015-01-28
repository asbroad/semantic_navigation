#! /usr/bin/env python

import roslib; roslib.load_manifest('voice_command')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from voice_command.msg import PosName
import os

class update_dictionary():
    def __init__(self):
        rospy.init_node('update_location_dictionary')
        rospy.Subscriber('voice_command/set', PosName, self.update_dict_callback)
        self.has_pose = False
        self.current_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('youbot/amcl_pose', PoseWithCovarianceStamped, self.update_current_pose)

    def update_dict_callback(self,data):

        while not self.has_pose:
            rospy.sleep(1)

        location_name = data.pos_name

        script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
        rel_path = "../config/location_dictionary.yaml"
        abs_file_path = os.path.join(script_dir, rel_path)

        with open(abs_file_path, 'a') as myfile:
            myfile.write(location_name + ': {' + \
               'pos_x: ' + str(float(self.current_pose.pose.pose.position.x/1.0)) + ',' + \
               'pos_y: ' + str(float(self.current_pose.pose.pose.position.y/1.0)) + ',' + \
               'pos_z: ' + str(float(self.current_pose.pose.pose.position.z/1.0)) + ',' + \
               'orient_x: ' + str(float(self.current_pose.pose.pose.orientation.x/1.0)) + ',' + \
               'orient_y: ' + str(float(self.current_pose.pose.pose.orientation.y/1.0)) + ',' + \
               'orient_z: ' + str(float(self.current_pose.pose.pose.orientation.z/1.0)) + ',' + \
               'orient_w: ' + str(float(self.current_pose.pose.pose.orientation.w/1.0)) + '}\n')

        rospy.set_param(location_name + '/pos_x', float(self.current_pose.pose.pose.position.x/1.0))
        rospy.set_param(location_name + '/pos_y', float(self.current_pose.pose.pose.position.y/1.0))
        rospy.set_param(location_name + '/pos_z', float(self.current_pose.pose.pose.position.z/1.0))
        rospy.set_param(location_name + '/orient_x', float(self.current_pose.pose.pose.orientation.x/1.0))
        rospy.set_param(location_name + '/orient_y', float(self.current_pose.pose.pose.orientation.y/1.0))
        rospy.set_param(location_name + '/orient_z', float(self.current_pose.pose.pose.orientation.z/1.0))
        rospy.set_param(location_name + '/orient_w', float(self.current_pose.pose.pose.orientation.w/1.0))

        rospy.loginfo("Finished updating dictionary and parameter server.")


    def update_current_pose(self, current_pose):
        self.current_pose = current_pose
        self.has_pose = True

if __name__ == '__main__':
    try:
        update_dictionary()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished updating dictionary")
