#!/usr/bin/env python
"""测试 6c: 包含未知关节"""
import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    rospy.init_node('send_unknown_joint_goal')
    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    rospy.loginfo('等待 action server...')
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr('连接超时!')
        sys.exit(1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['arm_joint_1', 'unknown_joint', 'arm_joint_5']

    pt = JointTrajectoryPoint()
    pt.positions = [0.5, 0.0, 0.3]
    pt.velocities = [0.0, 0.0, 0.0]
    pt.time_from_start = rospy.Duration(1.0)
    goal.trajectory.points.append(pt)

    rospy.loginfo('发送包含未知关节的轨迹')
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(5.0))
    if not finished:
        rospy.logerr('执行超时!')
        client.cancel_goal()
    else:
        state = client.get_state()
        result = client.get_result()
        state_names = {2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED'}
        rospy.loginfo('结果: %s (%d)', state_names.get(state, 'UNKNOWN'), state)
        if result:
            rospy.loginfo('错误信息: %s', result.error_string)


if __name__ == '__main__':
    main()
