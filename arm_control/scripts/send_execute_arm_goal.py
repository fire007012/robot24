#!/usr/bin/env python3

import argparse
import sys

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped

from arm_control.msg import ExecuteArmGoalAction, ExecuteArmGoalGoal


def build_parser():
    parser = argparse.ArgumentParser(
        description="Smoke client for /arm_control/execute_goal"
    )
    parser.add_argument(
        "--action-name",
        dest="action_name",
        default=None,
        help="Override action server name. Defaults to ~action_name or /arm_control/execute_goal.",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for the action server.",
    )
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for the action result.",
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    named_parser = subparsers.add_parser("named", help="Send a named target goal.")
    named_parser.add_argument(
        "--name",
        default="ready",
        help="Named target defined in MoveIt/SRDF. Default: ready.",
    )

    joints_parser = subparsers.add_parser("joints", help="Send a full joint target goal.")
    joints_parser.add_argument(
        "--joint",
        dest="joints",
        action="append",
        default=[],
        help="Joint target in name=value form. Repeat for each joint.",
    )

    pose_parser = subparsers.add_parser("pose", help="Send a pose target goal.")
    pose_parser.add_argument("--frame", required=True, help="Pose frame id.")
    pose_parser.add_argument("--x", type=float, required=True, help="Position X.")
    pose_parser.add_argument("--y", type=float, required=True, help="Position Y.")
    pose_parser.add_argument("--z", type=float, required=True, help="Position Z.")
    pose_parser.add_argument(
        "--qx", type=float, default=0.0, help="Orientation quaternion x."
    )
    pose_parser.add_argument(
        "--qy", type=float, default=0.0, help="Orientation quaternion y."
    )
    pose_parser.add_argument(
        "--qz", type=float, default=0.0, help="Orientation quaternion z."
    )
    pose_parser.add_argument(
        "--qw", type=float, default=1.0, help="Orientation quaternion w."
    )

    return parser


def parse_joint_assignments(assignments):
    joint_map = {}
    for raw in assignments:
        if "=" not in raw:
            raise ValueError("invalid --joint value '{}'".format(raw))
        name, value_text = raw.split("=", 1)
        name = name.strip()
        value_text = value_text.strip()
        if not name:
            raise ValueError("joint name cannot be empty in '{}'".format(raw))
        try:
            value = float(value_text)
        except ValueError:
            raise ValueError("joint '{}' has invalid float '{}'".format(name, value_text))
        if name in joint_map:
            raise ValueError("duplicate joint '{}'".format(name))
        joint_map[name] = value

    if not joint_map:
        raise ValueError("at least one --joint name=value is required")

    return joint_map


def build_goal(args):
    goal = ExecuteArmGoalGoal()

    if args.mode == "named":
        goal.target_type = ExecuteArmGoalGoal.TARGET_NAMED
        goal.named_target = args.name
        return goal

    if args.mode == "joints":
        joint_map = parse_joint_assignments(args.joints)
        goal.target_type = ExecuteArmGoalGoal.TARGET_JOINTS
        goal.joint_names = list(joint_map.keys())
        goal.joint_positions = [joint_map[name] for name in goal.joint_names]
        return goal

    if args.mode == "pose":
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = args.frame
        pose.pose.position.x = args.x
        pose.pose.position.y = args.y
        pose.pose.position.z = args.z
        pose.pose.orientation.x = args.qx
        pose.pose.orientation.y = args.qy
        pose.pose.orientation.z = args.qz
        pose.pose.orientation.w = args.qw

        goal.target_type = ExecuteArmGoalGoal.TARGET_POSE
        goal.pose_target = pose
        return goal

    raise ValueError("unsupported mode '{}'".format(args.mode))


def resolve_action_name(cli_value):
    if cli_value:
        return cli_value
    return rospy.get_param("~action_name", "/arm_control/execute_goal")


def feedback_cb(feedback):
    phase = getattr(feedback, "phase", "")
    message = getattr(feedback, "message", "")
    rospy.loginfo("feedback phase=%s message=%s", phase, message)


def main():
    parser = build_parser()
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("send_execute_arm_goal", anonymous=True)
    action_name = resolve_action_name(args.action_name)

    try:
        goal = build_goal(args)
    except ValueError as exc:
        print("invalid arguments: {}".format(exc), file=sys.stderr)
        return 2

    client = actionlib.SimpleActionClient(action_name, ExecuteArmGoalAction)

    print("connecting to action server: {}".format(action_name))
    if not client.wait_for_server(rospy.Duration.from_sec(args.server_timeout)):
        print(
            "timed out waiting for action server after {:.3f}s".format(
                args.server_timeout
            ),
            file=sys.stderr,
        )
        return 3

    client.send_goal(goal, feedback_cb=feedback_cb)

    if not client.wait_for_result(rospy.Duration.from_sec(args.result_timeout)):
        client.cancel_goal()
        print(
            "timed out waiting for action result after {:.3f}s".format(
                args.result_timeout
            ),
            file=sys.stderr,
        )
        return 4

    result = client.get_result()
    if result is None:
        print("action completed without a result payload", file=sys.stderr)
        return 5

    print("success={}".format(result.success))
    print("error_code={}".format(result.error_code))
    print("message={}".format(result.message))
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
