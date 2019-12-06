#!/usr/bin/env python
# -*- coding: utf-8 -*-
## This code is useless, Use test_command_action_client.py

import rospy
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


if __name__ == "__main__":
    rospy.init_node('test_command_publisher')
    pub = rospy.Publisher('/ve026a/arm_controller/command', JointTrajectory, queue_size=1)

    rospy.loginfo("Turn on motors");
    rospy.wait_for_service('/ve026a/set_motor')
    try:
        res = rospy.ServiceProxy('/ve026a/set_motor', SetBool)(True)
        print(res.message)
    except rospy.ServiceException, e:
        print("Service call failed, %s"%e)

    msg = JointTrajectory();
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

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

    msg.points = [p1, p2, p3]

    r = rospy.Rate(10)
    for i in range(3):
        pub.publish(msg)
        r.sleep()

    rospy.loginfo("published")
    rospy.spin()
