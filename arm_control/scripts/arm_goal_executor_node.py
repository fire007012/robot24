#!/usr/bin/env python3

import sys
import threading

import actionlib
import moveit_commander
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from controller_manager_msgs.srv import ListControllers

from arm_control.msg import (
    ExecuteArmGoalAction,
    ExecuteArmGoalFeedback,
    ExecuteArmGoalGoal,
    ExecuteArmGoalResult,
)


class ArmGoalExecutorNode(object):
    TARGET_JOINTS = 1
    TARGET_NAMED = 2
    TARGET_POSE = 3

    ERROR_NONE = 0
    ERROR_INVALID_GOAL = 1
    ERROR_NOT_READY = 2
    ERROR_PREEMPTED = 6

    PHASE_VALIDATING = 1
    PHASE_WAITING_READY = 2
    PHASE_STOPPING = 5

    def __init__(self):
        self.action_name = rospy.get_param("~action_name", "/arm_control/execute_goal")
        self.group_name = rospy.get_param("~group_name", "arm")
        self.reference_frame = rospy.get_param("~reference_frame", "base_link")
        self.end_effector_link = rospy.get_param("~end_effector_link", "")
        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.2))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.2))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.num_planning_attempts = int(rospy.get_param("~num_planning_attempts", 1))
        self.ready_timeout = float(rospy.get_param("~ready_timeout", 2.0))
        self.controller_manager_ns = rospy.get_param(
            "~controller_manager_ns", "/controller_manager"
        ).rstrip("/")
        self.joint_state_controller = rospy.get_param(
            "~joint_state_controller", "joint_state_controller"
        )
        self.arm_controller = rospy.get_param("~arm_controller", "arm_position_controller")
        self.arm_action_ns = rospy.get_param(
            "~arm_action_ns", "/arm_position_controller/follow_joint_trajectory"
        )

        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group.set_pose_reference_frame(self.reference_frame)
        if self.end_effector_link:
            self.group.set_end_effector_link(self.end_effector_link)
        self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.num_planning_attempts)

        self.active_joints = list(self.group.get_active_joints())
        self.named_targets = set(self.group.get_named_targets())

        self.list_controllers_srv_name = (
            self.controller_manager_ns + "/list_controllers"
        )
        self.list_controllers = rospy.ServiceProxy(
            self.list_controllers_srv_name, ListControllers
        )
        self.arm_action_client = actionlib.SimpleActionClient(
            self.arm_action_ns, FollowJointTrajectoryAction
        )

        self._preempt_requested = threading.Event()
        self._group_lock = threading.RLock()

        self.server = actionlib.SimpleActionServer(
            self.action_name,
            ExecuteArmGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        rospy.loginfo(
            "arm_goal_executor_node started: action=%s group=%s ref=%s ee=%s",
            self.action_name,
            self.group_name,
            self.reference_frame,
            self.end_effector_link or "(MoveIt default)",
        )
        rospy.loginfo(
            "arm_goal_executor_node active_joints=%s named_targets=%s",
            self.active_joints,
            sorted(self.named_targets),
        )

    def _error_const(self, name, default):
        return getattr(ExecuteArmGoalResult, name, default)

    def _phase_const(self, name, default):
        return getattr(ExecuteArmGoalFeedback, name, default)

    def _target_const(self, name, default):
        return getattr(ExecuteArmGoalGoal, name, default)

    def publish_feedback(self, phase, message):
        feedback = ExecuteArmGoalFeedback()
        feedback.phase = phase
        feedback.message = message
        self.server.publish_feedback(feedback)

    def make_result(self, success, error_code, message):
        result = ExecuteArmGoalResult()
        result.success = bool(success)
        result.error_code = error_code
        result.message = message
        return result

    def set_aborted(self, error_name, default_error, message):
        result = self.make_result(
            False, self._error_const(error_name, default_error), message
        )
        self.server.set_aborted(result, message)

    def set_preempted(self, message):
        result = self.make_result(
            False,
            self._error_const("ERROR_PREEMPTED", self.ERROR_PREEMPTED),
            message,
        )
        self.server.set_preempted(result, message)

    def preempt_cb(self):
        self._preempt_requested.set()
        self.publish_feedback(
            self._phase_const("PHASE_STOPPING", self.PHASE_STOPPING),
            "preempt requested; stopping MoveIt group",
        )
        try:
            with self._group_lock:
                self.group.stop()
        except Exception as exc:
            rospy.logwarn("failed to stop MoveIt group on preempt: %s", exc)

    def execute_cb(self, goal):
        self._preempt_requested.clear()
        self.publish_feedback(
            self._phase_const("PHASE_VALIDATING", self.PHASE_VALIDATING),
            "validating arm goal",
        )

        target_type = int(goal.target_type)
        supported_targets = {
            self._target_const("TARGET_JOINTS", self.TARGET_JOINTS),
            self._target_const("TARGET_NAMED", self.TARGET_NAMED),
            self._target_const("TARGET_POSE", self.TARGET_POSE),
        }
        if target_type not in supported_targets:
            self.set_aborted(
                "ERROR_INVALID_GOAL",
                self.ERROR_INVALID_GOAL,
                "unknown target_type=%s" % target_type,
            )
            return

        if self.check_preempt("preempted before readiness check"):
            return

        self.publish_feedback(
            self._phase_const("PHASE_WAITING_READY", self.PHASE_WAITING_READY),
            "checking controller readiness",
        )
        ready, ready_msg = self.check_ready()
        if not ready:
            self.set_aborted("ERROR_NOT_READY", self.ERROR_NOT_READY, ready_msg)
            return

        self.set_aborted(
            "ERROR_INVALID_GOAL",
            self.ERROR_INVALID_GOAL,
            "target execution is not implemented yet in this skeleton",
        )

    def check_preempt(self, message):
        if not self._preempt_requested.is_set() and not self.server.is_preempt_requested():
            return False

        self.publish_feedback(
            self._phase_const("PHASE_STOPPING", self.PHASE_STOPPING), message
        )
        try:
            with self._group_lock:
                self.group.stop()
                self.group.clear_pose_targets()
        except Exception as exc:
            rospy.logwarn("failed to cleanly stop MoveIt group after preempt: %s", exc)
        self.set_preempted(message)
        return True

    def check_ready(self):
        timeout = rospy.Duration(self.ready_timeout)
        try:
            rospy.wait_for_service(self.list_controllers_srv_name, timeout.to_sec())
        except rospy.ROSException as exc:
            return (
                False,
                "controller manager service %s unavailable: %s"
                % (self.list_controllers_srv_name, exc),
            )

        try:
            response = self.list_controllers()
        except rospy.ServiceException as exc:
            return (
                False,
                "failed to call %s: %s" % (self.list_controllers_srv_name, exc),
            )

        states = {controller.name: controller.state for controller in response.controller}
        for name in (self.joint_state_controller, self.arm_controller):
            state = states.get(name)
            if state != "running":
                return (
                    False,
                    "controller %s is not running; current state=%s"
                    % (name, state or "<missing>"),
                )

        if not self.arm_action_client.wait_for_server(timeout):
            return (
                False,
                "arm action server %s unavailable within %.3fs"
                % (self.arm_action_ns, self.ready_timeout),
            )

        return True, "ready"


def main():
    rospy.init_node("arm_goal_executor_node")
    ArmGoalExecutorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
