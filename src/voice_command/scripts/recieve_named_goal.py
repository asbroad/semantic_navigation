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

class comprehend_goal():
    def __init__(self):
        rospy.init_node('recieve_named_goal')
        rospy.Subscriber('voice_command/move', PosName, self.goal_name_callback)

    def goal_name_callback(self,data):

        goal_name = str(data.pos_name)

        if rospy.has_param(goal_name):
           # How long in seconds should the robot pause at each location?
           self.rest_time = rospy.get_param("~rest_time", 10)
           # Are we running in the fake simulator?
           self.fake_test = rospy.get_param("~fake_test", False)
           # Goal state return values
           goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

           # Set up the goal locations. Poses are defined in the map frame.
           locations = dict()
           full_goal_pos = rospy.get_param(goal_name)
           locations[goal_name] = Pose(Point(float(full_goal_pos.get('pos_x')), float(full_goal_pos.get('pos_y')), float(full_goal_pos.get('pos_z'))), Quaternion(float(full_goal_pos.get('orient_x')), float(full_goal_pos.get('orient_y')), float(full_goal_pos.get('orient_z')), float(full_goal_pos.get('orient_w'))))

           # Publisher to manually control the robot (e.g. to stop it)
           self.cmd_vel_pub = rospy.Publisher('youbot/cmd_vel', Twist)

           # Subscribe to the move_base action server
           self.move_base = actionlib.SimpleActionClient("youbot/move_base", MoveBaseAction)

           rospy.loginfo("Waiting for move_base action server...")

           # Wait 60 seconds for the action server to become available
           self.move_base.wait_for_server(rospy.Duration(60))

           rospy.loginfo("Connected to move base server")

           # A variable to hold the initial pose of the robot to be set by
           # the user in RViz
           initial_pose = PoseWithCovarianceStamped()

           # Variables to keep track of success rate, running time,
           # and distance traveled
           n_locations = len(locations)
           n_goals = 0
           n_successes = 0
           i = n_locations
           distance_traveled = 0
           start_time = rospy.Time.now()
           running_time = 0
           location = ""
           last_location = ""

           # Get the initial pose from the user
           self.last_location = Pose()
           rospy.Subscriber('youbot/amcl_pose', PoseWithCovarianceStamped, self.update_initial_pose)

           # Make sure we have the initial pose
           while initial_pose.header.stamp == "":
              rospy.sleep(1)

           rospy.loginfo("Starting navigation test")

           # Get the next location in the current sequence
           location = goal_name

           # Keep track of the distance traveled.
           # Use updated initial pose if available.
           if initial_pose.header.stamp == "":
               distance = sqrt(pow(locations[location].position.x -
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
           else:
               rospy.loginfo("Updating current pose.")
               distance = sqrt(pow(locations[location].position.x -
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
               initial_pose.header.stamp = ""

           # Store the last location for distance calculations
           last_location = location

           # Increment the counters
           i += 1
           n_goals += 1

           # Set up the next goal location
           self.goal = MoveBaseGoal()
           self.goal.target_pose.pose = locations[location]
           self.goal.target_pose.header.frame_id = 'map'
           self.goal.target_pose.header.stamp = rospy.Time.now()

           # Let the user know where the robot is going next
           rospy.loginfo("Going to: " + str(location))

           # Start the robot toward the next location
           self.move_base.send_goal(self.goal)

           # Allow 5 minutes to get there
           finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

           # Check for success or failure
           if not finished_within_time:
               self.move_base.cancel_goal()
               rospy.loginfo("Timed out achieving goal")
           else:
               state = self.move_base.get_state()
               if state == GoalStatus.SUCCEEDED:
                   rospy.loginfo("Goal succeeded!")
                   n_successes += 1
                   distance_traveled += distance
                   rospy.loginfo("State:" + str(state))
               else:
                   rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

           # How long have we been running?
           running_time = rospy.Time.now() - start_time
           running_time = running_time.secs / 60.0

           # Print a summary success/failure, distance traveled and time elapsed
           rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")
           rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
           rospy.sleep(self.rest_time)
        else:
           print('The parameter server is not aware of a location named : ' + goal_name)

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        comprehend_goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished.")
