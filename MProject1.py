#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
 
from std_msgs.msg import String
from gazebo_msgs import msg 

from open_grip import open
from close_grip import close
from fetch import fetch
from move_to import move_to_cube, move_to_bucket, move_up

def get_cube_seq(cubes):
    open()
    move_to_cube(cubes.pose[3])
    close()
    move_up()
    move_to_bucket()
    open()

if __name__=='__main__':
  try:
    cubes=fetch()
    get_cube_seq(cubes)
  except rospy.ROSInterruptException:
    pass