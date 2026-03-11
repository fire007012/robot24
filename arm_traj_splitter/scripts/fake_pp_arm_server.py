#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
)


class FakePPArmServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            '/pp_arm_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute,
            auto_start=False,
        )
        self.behavior = rospy.get_param('~behavior', 'succeed')
        self.exec_time = rospy.get_param('~exec_time', 0.5)
        self.server.start()
        rospy.loginfo('[FakePP] 启动, behavior=%s, exec_time=%.1f', self.behavior, self.exec_time)

    def execute(self, goal):
        names = goal.trajectory.joint_names
        n_points = len(goal.trajectory.points)
        rospy.loginfo('[FakePP] 收到轨迹: joints=%s, points=%d', names, n_points)

        for i, pt in enumerate(goal.trajectory.points):
            rospy.loginfo('[FakePP]   point[%d] t=%.3f pos=%s',
                          i, pt.time_from_start.to_sec(),
                          ['%.4f' % p for p in pt.positions])

        if self.behavior == 'abort':
            rospy.logwarn('[FakePP] 模拟 ABORT')
            self.server.set_aborted(FollowJointTrajectoryResult(error_code=-1))
            return

        if self.behavior == 'timeout':
            rospy.logwarn('[FakePP] 模拟超时, 不返回结果...')
            rospy.sleep(60.0)
            return

        rate = rospy.Rate(10)
        elapsed = 0.0
        dt = 0.1
        while elapsed < self.exec_time:
            if self.server.is_preempt_requested():
                rospy.logwarn('[FakePP] 收到 CANCEL')
                self.server.set_preempted()
                return
            fb = FollowJointTrajectoryFeedback()
            fb.joint_names = names
            self.server.publish_feedback(fb)
            rate.sleep()
            elapsed += dt

        rospy.loginfo('[FakePP] 执行完成, 返回 SUCCEEDED')
        self.server.set_succeeded(FollowJointTrajectoryResult(error_code=0))


if __name__ == '__main__':
    rospy.init_node('fake_pp_arm_server')
    FakePPArmServer()
    rospy.spin()
