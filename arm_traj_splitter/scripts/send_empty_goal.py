#!/usr/bin/env python
"""测试 6a: 空轨迹"""
import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


def main():
    rospy.init_node('send_empty_goal')
    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    rospy.loginfo('等待 action server...')
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr('连接超时!')
        sys.exit(1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = []
    goal.trajectory.points = []

    rospy.loginfo('发送空轨迹')
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
