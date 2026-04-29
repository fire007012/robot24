#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperCmdNode(object):
    def __init__(self):
        self.closed_reference = float(
            rospy.get_param("~closed_reference_position", 4.808646)
        )
        self.default_closed_reference = self.closed_reference
        self.stroke = float(rospy.get_param("~stroke", 0.06))
        self.position_topic = rospy.get_param(
            "~position_topic", "/arm_control/gripper_position"
        )
        self.open_close_topic = rospy.get_param(
            "~open_close_topic", "/arm_control/gripper_open"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/gripper_controller/command"
        )
        self.joint_name = rospy.get_param("~joint_name", "left_gripper_finger_joint")
        self.min_pos = float(rospy.get_param("~min_position", 0.0))
        self.max_pos = float(rospy.get_param("~max_position", 0.044))
        self.open_pos = float(rospy.get_param("~open_position", 0.044))
        self.close_pos = float(rospy.get_param("~close_position", 0.0))
        self.duration = float(rospy.get_param("~move_duration", 0.8))
        self.auto_close_on_start = bool(rospy.get_param("~auto_close_on_start", False))
        self.capture_closed_reference_from_joint_state = bool(
            rospy.get_param("~capture_closed_reference_from_joint_state", True)
        )
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self.startup_command_delay = float(
            rospy.get_param("~startup_command_delay", 1.0)
        )
        self.startup_retry_period = float(
            rospy.get_param("~startup_retry_period", 0.5)
        )
        self.startup_retry_count = int(
            rospy.get_param("~startup_retry_count", 10)
        )
        self.lifecycle_state_topic = rospy.get_param(
            "~lifecycle_state_topic", "/hybrid_motor_hw_node/lifecycle_state"
        )
        self.require_running_lifecycle = bool(
            rospy.get_param("~require_running_lifecycle", True)
        )
        self._startup_retries = 0
        self._lifecycle_state = ""
        self._closed_reference_captured = False

        self.pub = rospy.Publisher(self.output_topic, JointTrajectory, queue_size=10)
        self.sub_pos = rospy.Subscriber(
            self.position_topic, Float64, self.pos_cb, queue_size=10
        )
        self.sub_open = rospy.Subscriber(
            self.open_close_topic, Bool, self.open_cb, queue_size=10
        )
        self.sub_lifecycle = rospy.Subscriber(
            self.lifecycle_state_topic, String, self.lifecycle_cb, queue_size=10
        )
        self.sub_joint_states = rospy.Subscriber(
            self.joint_states_topic, JointState, self.joint_states_cb, queue_size=10
        )

        rospy.loginfo(
            "gripper_cmd_node started: %s -> %s",
            self.position_topic,
            self.output_topic,
        )
        rospy.loginfo(
            "gripper_cmd_node open/close topic: %s", self.open_close_topic
        )
        rospy.loginfo(
            "gripper_cmd_node absolute range: [%.6f, %.6f]",
            self.closed_reference - self.stroke,
            self.closed_reference,
        )
        if self.capture_closed_reference_from_joint_state:
            rospy.loginfo(
                "gripper_cmd_node will capture startup closed reference for %s from %s",
                self.joint_name,
                self.joint_states_topic,
            )
        if self.require_running_lifecycle:
            rospy.loginfo(
                "gripper_cmd_node waits for lifecycle Running on %s before startup close",
                self.lifecycle_state_topic,
            )

        if self.auto_close_on_start:
            rospy.Timer(
                rospy.Duration.from_sec(self.startup_command_delay),
                self._startup_close_cb,
                oneshot=True,
            )

    def lifecycle_cb(self, msg):
        self._lifecycle_state = msg.data.strip()

    def joint_states_cb(self, msg):
        if not self.capture_closed_reference_from_joint_state or self._closed_reference_captured:
            return

        try:
            joint_index = msg.name.index(self.joint_name)
        except ValueError:
            return

        if joint_index >= len(msg.position):
            return

        position = msg.position[joint_index]
        if not isinstance(position, float):
            position = float(position)
        if not rospy.is_shutdown():
            self.closed_reference = position
            self._closed_reference_captured = True
            rospy.loginfo(
                "gripper_cmd_node captured startup closed reference %.6f rad for %s from %s",
                self.closed_reference,
                self.joint_name,
                self.joint_states_topic,
            )

    def clamp(self, val):
        return max(self.min_pos, min(self.max_pos, val))

    def to_absolute_position(self, opening):
        opening = self.clamp(opening)
        return self.closed_reference - opening

    def publish_target(self, target):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.to_absolute_position(target)]
        point.time_from_start = rospy.Duration.from_sec(self.duration)

        traj.points = [point]
        self.pub.publish(traj)

    def _startup_close_cb(self, _event):
        if rospy.is_shutdown():
            return

        if (
            self.require_running_lifecycle
            and self._lifecycle_state
            and self._lifecycle_state != "Running"
        ):
            self._startup_retries += 1
            if self._startup_retries >= self.startup_retry_count:
                rospy.logwarn(
                    "gripper_cmd_node startup close skipped after %d retries: lifecycle stayed at %s",
                    self._startup_retries,
                    self._lifecycle_state,
                )
                return

            rospy.logwarn_throttle(
                2.0,
                "gripper_cmd_node waiting for lifecycle Running on %s, current=%s",
                self.lifecycle_state_topic,
                self._lifecycle_state,
            )
            rospy.Timer(
                rospy.Duration.from_sec(self.startup_retry_period),
                self._startup_close_cb,
                oneshot=True,
            )
            return

        if self.require_running_lifecycle and not self._lifecycle_state:
            self._startup_retries += 1
            if self._startup_retries >= self.startup_retry_count:
                rospy.logwarn(
                    "gripper_cmd_node startup close skipped after %d retries: no lifecycle state on %s",
                    self._startup_retries,
                    self.lifecycle_state_topic,
                )
                return

            rospy.logwarn_throttle(
                2.0,
                "gripper_cmd_node waiting for first lifecycle state on %s before startup close",
                self.lifecycle_state_topic,
            )
            rospy.Timer(
                rospy.Duration.from_sec(self.startup_retry_period),
                self._startup_close_cb,
                oneshot=True,
            )
            return

        if self.pub.get_num_connections() <= 0:
            self._startup_retries += 1
            if self._startup_retries >= self.startup_retry_count:
                rospy.logwarn(
                    "gripper_cmd_node startup close skipped after %d retries: no subscribers on %s",
                    self._startup_retries,
                    self.output_topic,
                )
                return

            rospy.logwarn_throttle(
                2.0,
                "gripper_cmd_node waiting for controller subscriber on %s before startup close",
                self.output_topic,
            )
            rospy.Timer(
                rospy.Duration.from_sec(self.startup_retry_period),
                self._startup_close_cb,
                oneshot=True,
            )
            return

        rospy.loginfo(
            "gripper_cmd_node sending startup close target %.6f to %s",
            self.close_pos,
            self.output_topic,
        )
        self.publish_target(self.close_pos)

    def pos_cb(self, msg):
        self.publish_target(msg.data)

    def open_cb(self, msg):
        self.publish_target(self.open_pos if msg.data else self.close_pos)


if __name__ == "__main__":
    rospy.init_node("gripper_cmd_node")
    GripperCmdNode()
    rospy.spin()
