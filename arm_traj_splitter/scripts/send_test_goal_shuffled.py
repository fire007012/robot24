#!/usr/bin/env python
"""测试 2: 关节顺序打乱 - 验证按名字匹配而非索引"""
import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def make_goal(joint_names, positions_list, times):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    for pos, t in zip(positions_list, times):
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.velocities = [0.0] * len(pos)
        pt.time_from_start = rospy.Duration(t)
        goal.trajectory.points.append(pt)
    return goal


def main():
    rospy.init_node('send_test_goal_shuffled')

    action_name = '/arm_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

    rospy.loginfo('等待 action server: %s', action_name)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr('连接超时!')
        sys.exit(1)

    # 关节顺序打乱: 5,2,6,1,4,3
    names = [
        'arm_joint_5', 'arm_joint_2', 'arm_joint_6',
        'arm_joint_1', 'arm_joint_4', 'arm_joint_3',
    ]
    # 位置值对应: [j5, j2, j6, j1, j4, j3]
    waypoints = [
        [0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
        [0.4,  0.3, -0.3,  0.5,  0.1, -0.2],
        [0.8,  0.6, -0.6,  1.0,  0.2, -0.4],
    ]
    times = [0.0, 1.5, 3.0]

    rospy.loginfo('发送打乱顺序的 6 轴轨迹')
    rospy.loginfo('  关节顺序: %s', names)
    goal = make_goal(names, waypoints, times)
    client.send_goal(goal)

    rospy.loginfo('等待结果...')
    finished = client.wait_for_result(rospy.Duration(10.0))

    if not finished:
        rospy.logerr('执行超时!')
        client.cancel_goal()
    else:
        state = client.get_state()
        state_names = {2: 'PREEMPTED', 3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED'}
        rospy.loginfo('结果: %s (%d)', state_names.get(state, 'UNKNOWN'), state)


if __name__ == '__main__':
    main()
