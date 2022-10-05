#!/usr/bin/env python
from gazebo_msgs.msg import ModelStates
import rospy
import roslib

def fetch_model_states():
    model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, 5)
    return model_states
