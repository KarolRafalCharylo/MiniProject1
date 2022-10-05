#!/usr/bin/env python
from gazebo_msgs import msg
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
roslib.load_manifest('hello_ros')


def move_to_joint(objective):
    # BEGIN_TUTORIAL
    # First initialize moveit_commander and rospy.
    print("============ Starting tutorial setup")
    
    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    display_trajectory_publisher = []

    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    # print("============ Starting tutorial ")
    # # We can get the name of the reference frame for this robot
    # print("============ Reference frame: %s" % group.get_planning_frame())
    # # We can also print(the name of the end-effector link for this group)
    # print("============ End effector frame: %s" %
    #       group.get_end_effector_link())
    # # We can get a list of all the groups in the robot
    # print("============ Robot Groups:")
    # print(robot.get_group_names())
    # # Sometimes for debugging it is useful to print(the entire state of the)
    # # robot.
    # print("============ Printing robot state")
    # print(robot.get_current_state())
    # print("============")

    # Bucket avoidance
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.581763
    p.pose.position.y = -0.207290
    p.pose.position.z = 0.7
    conv_quat = tf_conversions.transformations.quaternion_from_euler(-0.071061, -0.000003, 0.785914)
    p.pose.orientation.x = conv_quat[0]
    p.pose.orientation.y = conv_quat[1]
    p.pose.orientation.z = conv_quat[2]
    p.pose.orientation.w = conv_quat[3]
    scene.add_box("bucket", p, (0.3, 0.3, 0.3))

    # Let's setup the planner
    # group.set_planning_time(0.0)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_num_planning_attempts(100)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    print("============ Generating plan 1")

    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = objective.position.x
    pose_goal.position.y = objective.position.y
    pose_goal.position.z = objective.position.z
    #print(pose_goal)
    group.set_pose_target(pose_goal)

    # Now, we call the planner to compute the plan
    plan1 = group.plan()

    print("============ Waiting while RVIZ displays plan1...")
    rospy.sleep(0.5)

    #display_trajectory = []

    # You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print("============ Visualizing plan1")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(group.plan())
    display_trajectory_publisher.publish(display_trajectory)
    print("============ Waiting while plan1 is visualized (again)...")
    rospy.sleep(2.)

    # If we're coming from another script we might want to remove the objects
    if "table" in scene.get_known_object_names():
        scene.remove_world_object("table")
    if "table2" in scene.get_known_object_names():
        scene.remove_world_object("table2")
    if "groundplane" in scene.get_known_object_names():
        scene.remove_world_object("groundplane")

    # Moving to a pose goal
    group.go(wait=True)
    rospy.sleep(1.)




def move_to_cartesian(*objectives):

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    display_trajectory_publisher = []

    # trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    # Let's setup the planner
    # group.set_planning_time(0.0)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_num_planning_attempts(100)
    group.set_max_velocity_scaling_factor(0.4)
    group.set_max_acceleration_scaling_factor(0.4)

    print("============ Generating plan 1")

    pose_goal = group.get_current_pose().pose
    waypoints = []
    waypoints.append(pose_goal)

    for objective in objectives:
        pose_goal.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
        pose_goal.position.x = objective.position.x
        pose_goal.position.y = objective.position.y
        pose_goal.position.z = objective.position.z
        print(pose_goal)

        # Create waypoints
        waypoints.append(pose_goal)
    print(waypoints)
    # createcartesian  plan
    (planbucket, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # eef_step
        0.0)         # jump_threshold
    #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

    #print("============ Waiting while RVIZ displays plan1...")
    rospy.sleep(0.5)

    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print("============ Visualizing plan1")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(planbucket)
    display_trajectory_publisher.publish(display_trajectory);
    print("============ Waiting while plan1 is visualized (again)...")
    rospy.sleep(2.)

    # If we're coming from another script we might want to remove the objects
    if "table" in scene.get_known_object_names():
        scene.remove_world_object("table")
    if "table2" in scene.get_known_object_names():
        scene.remove_world_object("table2")
    if "groundplane" in scene.get_known_object_names():
        scene.remove_world_object("groundplane")

    # Moving to a pose goal
    group.execute(planbucket, wait=True)
    rospy.sleep(1.)

