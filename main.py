#!/usr/bin/env python
from gazebo_msgs import msg, srv
from std_msgs.msg import String
import math
import shape_msgs.msg as shape_msgs
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import tf_conversions
import rospy
import copy
import sys
import roslib
import time

from fetch_model_states import fetch_model_states
from extract_by_name import extract_by_name
from close_grip import close
from open_grip import open
from move_to import move_to_joint, move_to_cartesian

roslib.load_manifest('hello_ros')

if __name__ == '__main__':
    try:
        rospy.init_node("miniproject1")
        
        model_states = fetch_model_states()
        cubes = extract_by_name(model_states, starts_with="cube")
        bucket = extract_by_name(model_states, starts_with="bucket")[0]
        bucket[1].position.z = 1.3
        bucket = bucket[1]

        moveit_commander.roscpp_initialize(sys.argv)
        for cube in cubes:
            print('Attempting pickup of {}'.format(cube[0]))
            open()
            rospy.sleep(1)
            
            above_cube = cube[1]
            above_cube.position.z = 1.1



            move_to_joint(above_cube)
            the_cube = cube[1]
            the_cube.position.z = 0.93
            move_to_cartesian(the_cube)
            close()
            rospy.sleep(1)
            above_cube = cube[1]
            above_cube.position.z = 1.3
            move_to_cartesian(above_cube)
            move_to_cartesian(bucket)
            #move_to_cartesian(above_cube, bucket)
            open()
            time.sleep(1)

        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass
