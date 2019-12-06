#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import SetBool
import actionlib

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance

if __name__ == "__main__":
    rospy.init_node('test_command_action_client')

    rospy.loginfo("Turn on motors");
    rospy.wait_for_service('/ve026a/set_motor')
    try:
        res = rospy.ServiceProxy('/ve026a/set_motor', SetBool)(True)
        print(res.message)
    except rospy.ServiceException, e:
        print("Service call failed, %s"%e)

    client = actionlib.SimpleActionClient('/ve026a/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    # fill data
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    p1 = JointTrajectoryPoint()
    p1.positions = [0, 0, 0, 0, 0, 0]
    p1.velocities = [0, 0, 0, 0, 0, 0]
    p1.time_from_start = rospy.Duration.from_sec(3.0)

    p2 = JointTrajectoryPoint()
    p2.positions = [0.5, 0.5, 0, 0, 0, 0]
    p2.velocities = [0, 0, 0, 0, 0, 0]
    p2.time_from_start = rospy.Duration.from_sec(5.0)

    p3 = JointTrajectoryPoint()
    p3.positions = [0, 0, 0, 0, 0, 0]
    p3.velocities = [0, 0, 0, 0, 0, 0]
    p3.time_from_start = rospy.Duration.from_sec(7.0)

    goal.trajectory.points = [p1, p2, p3]

    pt1 = JointTolerance()
    pt1.name = 'joint_1'
    pt1.position = 10.1
    pt1.velocity = 10.1
    pt1.acceleration = 10.1
    pt2 = JointTolerance()
    pt2.name = 'joint_2'
    pt2.position = 10.1
    pt2.velocity = 10.1
    pt2.acceleration = 10.1
    pt3 = JointTolerance()
    pt3.name = 'joint_3'
    pt3.position = 10.1
    pt3.velocity = 10.1
    pt3.acceleration = 10.1
    pt4 = JointTolerance()
    pt4.name = 'joint_4'
    pt4.position = 10.1
    pt4.velocity = 10.1
    pt4.acceleration = 10.1
    pt5 = JointTolerance()
    pt5.name = 'joint_5'
    pt5.position = 10.1
    pt5.velocity = 10.1
    pt5.acceleration = 10.1
    pt6 = JointTolerance()
    pt6.name = 'joint_6'
    pt6.position = 10.1
    pt6.velocity = 10.1
    pt6.acceleration = 0.1

    goal.path_tolerance = [pt1, pt2, pt3, pt4, pt5, pt6]
    goal.goal_tolerance = [pt1, pt2, pt3, pt4, pt5, pt6]

    goal.goal_time_tolerance = rospy.Duration.from_sec(10)

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(100.0))
    print(goal)

    rospy.spin()
