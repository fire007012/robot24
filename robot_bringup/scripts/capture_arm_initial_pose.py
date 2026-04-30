#!/usr/bin/env python3
"""Capture current arm joint states and write them as the default initial pose."""

import argparse
import os
import re
import sys

import rospy
import rosgraph
import rospkg
from sensor_msgs.msg import JointState


ARM_JOINTS = [
    "shoulder_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
]


def default_paths():
    rospack = rospkg.RosPack()
    moveit_config = rospack.get_path("car_moveit_config")
    bringup = rospack.get_path("robot_bringup")
    return {
        "srdf": [
            os.path.join(moveit_config, "config", "car_urdf.srdf"),
            os.path.join(moveit_config, "config", "car_urdf_gazebo.srdf"),
        ],
        "launch": [
            os.path.join(moveit_config, "launch", "gazebo.launch"),
            os.path.join(bringup, "launch", "full_system_simulate.launch"),
        ],
    }


def format_value(value):
    return ("%.6f" % value).rstrip("0").rstrip(".")


def capture_joint_positions(topic, timeout):
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    positions = dict(zip(msg.name, msg.position))
    missing = [joint for joint in ARM_JOINTS if joint not in positions]
    if missing:
        raise RuntimeError(
            "missing arm joints from %s: %s" % (topic, ", ".join(missing))
        )
    return {joint: float(positions[joint]) for joint in ARM_JOINTS}


def validate_positions(positions, allow_zero_pose):
    if allow_zero_pose:
        return
    if all(abs(positions[joint]) < 1e-9 for joint in ARM_JOINTS):
        raise RuntimeError(
            "captured all-zero main_arm pose; call /hybrid_motor_hw_node/init first "
            "and verify /joint_states contains real motor feedback"
        )


def update_srdf(path, positions, pose_name):
    with open(path, "r", encoding="utf-8") as stream:
        content = stream.read()

    state_pattern = re.compile(
        r'(<group_state\s+name="%s"\s+group="main_arm">)(.*?)(\s*</group_state>)'
        % re.escape(pose_name),
        re.DOTALL,
    )
    match = state_pattern.search(content)
    if match is None:
        raise RuntimeError("%s has no main_arm group_state named %s" % (path, pose_name))

    body = match.group(2)
    for joint_name in ARM_JOINTS:
        joint_pattern = re.compile(
            r'(<joint\s+name="%s"\s+value=")[^"]*("\s*/>)' % re.escape(joint_name)
        )
        body, count = joint_pattern.subn(
            r"\g<1>%s\2" % format_value(positions[joint_name]), body, count=1
        )
        if count != 1:
            raise RuntimeError("%s group_state %s is missing %s" % (path, pose_name, joint_name))

    updated = content[: match.start(2)] + body + content[match.end(2) :]
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(updated)


def build_spawn_args(positions):
    return " " + " ".join(
        "-J %s %s" % (joint_name, format_value(positions[joint_name]))
        for joint_name in ARM_JOINTS
    )


def update_spawn_args(existing_args, positions):
    updated_args = existing_args
    for joint_name in ARM_JOINTS:
        joint_pattern = re.compile(r"(-J\s+%s\s+)[^\s]+" % re.escape(joint_name))
        updated_args, count = joint_pattern.subn(
            r"\g<1>%s" % format_value(positions[joint_name]), updated_args, count=1
        )
        if count == 0:
            updated_args += " -J %s %s" % (joint_name, format_value(positions[joint_name]))
    return updated_args


def update_launch_initial_positions(path, positions):
    with open(path, "r", encoding="utf-8") as stream:
        content = stream.read()

    arg_pattern = re.compile(r'(name="initial_joint_positions"\s+default=")([^"]*)(")')
    match = arg_pattern.search(content)
    if match is None:
        raise RuntimeError("%s has no initial_joint_positions arg" % path)

    replacement = match.group(1) + update_spawn_args(match.group(2), positions) + match.group(3)
    updated = content[: match.start()] + replacement + content[match.end() :]
    count = 1
    if count != 1:
        raise RuntimeError("%s has no unique initial_joint_positions arg" % path)

    with open(path, "w", encoding="utf-8") as stream:
        stream.write(updated)


def print_positions(positions):
    rospy.loginfo("captured current main_arm pose:")
    for joint_name in ARM_JOINTS:
        rospy.loginfo("  %s: %s", joint_name, format_value(positions[joint_name]))


def main():
    parser = argparse.ArgumentParser(
        description="Capture /joint_states and update MoveIt/Gazebo initial arm pose."
    )
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--pose-name", default="up")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-zero-pose", action="store_true")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if not rosgraph.is_master_online():
        sys.stderr.write(
            "ROS master is not running. Start the real robot bringup in another terminal first, for example:\n"
            "  roslaunch robot_bringup full_system.launch enable_moveit:=false enable_rviz:=false "
            "enable_flipper_control:=false enable_mobility_control:=false\n"
        )
        return 1

    rospy.init_node("capture_arm_initial_pose", anonymous=True)
    positions = capture_joint_positions(args.topic, args.timeout)
    validate_positions(positions, args.allow_zero_pose)
    print_positions(positions)

    if args.dry_run:
        rospy.loginfo("dry-run requested; no files were changed")
        return 0

    paths = default_paths()
    for path in paths["srdf"]:
        update_srdf(path, positions, args.pose_name)
        rospy.loginfo("updated %s group_state %s", path, args.pose_name)
    for path in paths["launch"]:
        update_launch_initial_positions(path, positions)
        rospy.loginfo("updated %s initial_joint_positions", path)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSException, RuntimeError) as exc:
        rospy.logerr(str(exc))
        sys.exit(1)
