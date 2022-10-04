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
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil

def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg

def open():
    currentJointState = JointState()
    #rospy.init_node('test_publish')
 
  # Setup subscriber
  #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print 'Received!'
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    rate = rospy.Rate(10) # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        print 'Published!'
        rate.sleep()