#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('send_and_cancel')
    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)
    client.wait_for_server(rospy.Duration(5.0))

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'arm_joint_1', 'arm_joint_2', 'arm_joint_3',
        'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
    ]
    pt = JointTrajectoryPoint()
    pt.positions = [1.0] * 6
    pt.velocities = [0.0] * 6
    pt.time_from_start = rospy.Duration(5.0)
    goal.trajectory.points.append(pt)

    rospy.loginfo('发送 goal...')
    client.send_goal(goal)

    rospy.sleep(0.3)

    rospy.loginfo('发送 CANCEL!')
    client.cancel_goal()

    client.wait_for_result(rospy.Duration(3.0))
    state = client.get_state()
    rospy.loginfo('结果状态: %d (2=PREEMPTED)', state)


if __name__ == '__main__':
    main()
