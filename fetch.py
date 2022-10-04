#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
 
from std_msgs.msg import String

from gazebo_msgs import msg 


def mycallback(data):
        global cubes
        cubes = data
        #print('Function')
	
	# print the actual message in its raw format
	#rospy.loginfo("Here's what was subscribed: %s", data)
	
	# otherwise simply print a convenient message on the terminal
	#print('Data from /gazebo/model_state received')


def fetch():
	rospy.init_node('mylistener')
	rospy.Subscriber('gazebo/model_states', msg.ModelStates, mycallback, queue_size=1000)
	rospy.sleep (3.)
	# spin() simply keeps python from
	# exiting until this node is stopped
	#rospy.spin()
	return cubes

