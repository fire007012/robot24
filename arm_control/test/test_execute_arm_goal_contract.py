#!/usr/bin/env python3

import importlib
import unittest


def _try_import_action_symbols():
    """Import generated ROS action/message symbols if they are available."""
    try:
        action_module = importlib.import_module("arm_control.msg")
    except ImportError as exc:
        raise unittest.SkipTest(
            "arm_control.msg is not importable yet. Build the catkin workspace "
            "so ExecuteArmGoal action messages are generated before running "
            "this contract test. Original error: %s" % exc
        )

    required = [
        "ExecuteArmGoalAction",
        "ExecuteArmGoalGoal",
        "ExecuteArmGoalResult",
        "ExecuteArmGoalFeedback",
    ]
    missing = [name for name in required if not hasattr(action_module, name)]
    if missing:
        raise unittest.SkipTest(
            "Generated arm_control action symbols are incomplete: missing %s"
            % ", ".join(missing)
        )

    try:
        geometry_module = importlib.import_module("geometry_msgs.msg")
    except ImportError as exc:
        raise unittest.SkipTest(
            "geometry_msgs.msg is not importable in this environment. "
            "Source the workspace before running the contract test. "
            "Original error: %s" % exc
        )

    return (
        action_module.ExecuteArmGoalAction,
        action_module.ExecuteArmGoalGoal,
        action_module.ExecuteArmGoalResult,
        action_module.ExecuteArmGoalFeedback,
        geometry_module.PoseStamped,
    )


class ExecuteArmGoalContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (
            cls.ExecuteArmGoalAction,
            cls.ExecuteArmGoalGoal,
            cls.ExecuteArmGoalResult,
            cls.ExecuteArmGoalFeedback,
            cls.PoseStamped,
        ) = _try_import_action_symbols()

    def test_action_messages_are_constructible(self):
        self.assertIsNotNone(self.ExecuteArmGoalAction())
        self.assertIsNotNone(self.ExecuteArmGoalGoal())
        self.assertIsNotNone(self.ExecuteArmGoalResult())
        self.assertIsNotNone(self.ExecuteArmGoalFeedback())

    def test_named_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_NAMED
        goal.named_target = "ready"

        self.assertEqual(goal.target_type, goal.TARGET_NAMED)
        self.assertEqual(goal.named_target, "ready")
        self.assertEqual(list(goal.joint_names), [])
        self.assertEqual(list(goal.joint_positions), [])
        self.assertEqual(goal.pose_target.header.frame_id, "")

    def test_joint_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_JOINTS
        goal.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        goal.joint_positions = [0.0, -0.4, 0.8, 0.0, 0.5, 0.1]

        self.assertEqual(goal.target_type, goal.TARGET_JOINTS)
        self.assertEqual(len(goal.joint_names), 6)
        self.assertEqual(len(goal.joint_positions), 6)
        self.assertEqual(goal.named_target, "")
        self.assertEqual(goal.pose_target.header.frame_id, "")

    def test_pose_goal_contract(self):
        goal = self.ExecuteArmGoalGoal()
        goal.target_type = goal.TARGET_POSE
        goal.pose_target = self.PoseStamped()
        goal.pose_target.header.frame_id = "base_link"
        goal.pose_target.pose.orientation.w = 1.0
        goal.pose_target.pose.position.x = 0.30
        goal.pose_target.pose.position.y = 0.02
        goal.pose_target.pose.position.z = 0.25

        self.assertEqual(goal.target_type, goal.TARGET_POSE)
        self.assertEqual(goal.pose_target.header.frame_id, "base_link")
        self.assertAlmostEqual(goal.pose_target.pose.orientation.w, 1.0)
        self.assertEqual(goal.named_target, "")
        self.assertEqual(list(goal.joint_names), [])
        self.assertEqual(list(goal.joint_positions), [])

    def test_error_code_constants_exist(self):
        result = self.ExecuteArmGoalResult()
        expected = [
            "ERROR_NONE",
            "ERROR_INVALID_GOAL",
            "ERROR_NOT_READY",
            "ERROR_TF_FAILED",
            "ERROR_PLANNING_FAILED",
            "ERROR_EXECUTION_FAILED",
            "ERROR_PREEMPTED",
        ]

        for name in expected:
            self.assertTrue(
                hasattr(result, name),
                "missing ExecuteArmGoalResult constant %s" % name,
            )


if __name__ == "__main__":
    unittest.main()
