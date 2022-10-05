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
from move_to import move_to_cube, move_to_bucket, move_up
from close_grip import close
from open_grip import open

roslib.load_manifest('hello_ros')


def get_cube_seq(cubes):
    open()
    move_to_cube(cubes.pose[3])
    close()
    move_up()
    move_to_bucket()
    open()


if __name__ == '__main__':
    try:
        rospy.init_node("miniproject1")
        
        model_states = fetch_model_states()
        i = model_states.name.index("cube0")
        cubes = extract_by_name(model_states, starts_with="cube")
        bucket = extract_by_name(model_states, starts_with="bucket")[0]
        
        open()
        for cube in cubes:
            print('Attempting pickup of {}'.format(cube["name"]))
            move_to_cube(cube["pose"])
            close()
            time.sleep(1)
            open()
            pass
            
        # print(model_states.pose[i])
        # cubes = fetch()
        # get_cube_seq(cubes)
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
