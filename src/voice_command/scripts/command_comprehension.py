#! /usr/bin/env python

import roslib; roslib.load_manifest('voice_command')
import rospy
import actionlib
from actionlib_msgs.msg import *
from voice_command.msg import PosName

class translate_command():

    def __init__(self):
        rospy.init_node('command_comprehension')
        rospy.Subscriber('/recognizer/output', std_msgs.msg.String, self.speech_callback)
        self.move_pub = rospy.Publisher('/voice_command/move', PosName, queue_size=10)
        self.set_pub = rospy.Publisher('/voice_command/set', PosName, queue_size=10)
        self.map_pub = rospy.Publisher('/voice_command/map', std_msgs.msg.String, queue_size=10)

    def speech_callback(self, data):

        full_input = data.data

        move_commands = ['go to the','move to']
        set_commands = ['this is the', 'set position as']
        map_commands = ['update map']

        is_move_command = False
        is_set_command = False
        is_map_command = False
        loc_name = None

        for m_c in move_commands:
            if m_c in full_input:
                remove_command = full_input.replace(m_c,'')
                loc_name = remove_command.strip().replace(' ', '_')
                is_move_command = True
                break

        if not is_move_command:
            for s_c in set_commands:
                if s_c in full_input:
                    remove_command = full_input.replace(s_c,'')
                    loc_name = remove_command.strip().replace(' ', '_')
                    is_set_command = True
                    break

        if not is_move_command and not is_set_command:
            for mp_c in map_commands:
                if mp_c in full_input:
                    is_map_command = True
                    break

        if is_move_command:
            self.move_pub.publish(loc_name)
            rospy.loginfo('Recognized move command : ' + full_input + ' , location name : ' + loc_name)
        elif is_set_command:
            self.set_pub.publish(loc_name)
            rospy.loginfo('Recognized set command : ' + full_input + ' , location name : ' + loc_name)
        elif is_map_command:
            self.map_pub.publish('refresh')
            rospy.loginfo('Recognized map update command')
        else:
            rospy.loginfo('Not able to recognize either move, set or update command : ' + full_input)

if __name__ == '__main__':
    try:
        translate_command()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished translating command")
