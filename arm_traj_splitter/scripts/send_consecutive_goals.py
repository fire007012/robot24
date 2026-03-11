#!/usr/bin/env python
"""测试 7: 连续快速发送两个 goal，验证 preempt 逻辑"""
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def make_goal(goal_id, duration):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'arm_joint_1', 'arm_joint_2', 'arm_joint_3',
        'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
    ]
    pt = JointTrajectoryPoint()
    pt.positions = [float(goal_id)] * 6
    pt.velocities = [0.0] * 6
    pt.time_from_start = rospy.Duration(duration)
    goal.trajectory.points.append(pt)
    return goal


def main():
    rospy.init_node('send_consecutive_goals')
    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)

    rospy.loginfo('等待 action server...')
    client.wait_for_server(rospy.Duration(5.0))

    # 发送第一个 goal��长时间执行）
    goal1 = make_goal(1, 5.0)
    rospy.loginfo('发送 Goal 1 (5 秒执行时间)')
    client.send_goal(goal1)

    # 等待 0.5 秒，确保 goal1 已经开始执行
    rospy.sleep(0.5)

    # 立即发送第二个 goal（应该 preempt goal1）
    goal2 = make_goal(2, 2.0)
    rospy.loginfo('发送 Goal 2 (应该 preempt Goal 1)')
    client.send_goal(goal2)

    # 等待 goal2 完成
    rospy.loginfo('等待 Goal 2 完成...')
    finished = client.wait_for_result(rospy.Duration(10.0))

    if not finished:
        rospy.logerr('Goal 2 执行超时!')
        client.cancel_goal()
    else:
        state = client.get_state()
        state_names = {2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED'}
        rospy.loginfo('Goal 2 结果: %s (%d)', state_names.get(state, 'UNKNOWN'), state)


if __name__ == '__main__':
    main()
