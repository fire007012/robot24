#!/usr/bin/env python3

from __future__ import annotations

import argparse
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rospy
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from Eyou_ROS1_Master.msg import JointRuntimeStateArray
from Eyou_ROS1_Master.srv import SetJointMode
from flipper_control.msg import FlipperControlState
from flipper_control.srv import SetControlProfile, SetLinkageMode


DEFAULT_FLIPPER_NS = "/flipper_control"
DEFAULT_HYBRID_NS = "/hybrid_motor_hw_node"
DEFAULT_JOINT_NAMES = [
    "left_front_arm_joint",
    "right_front_arm_joint",
    "left_rear_arm_joint",
    "right_rear_arm_joint",
]
PROFILE_OPTIONS = ["csp_position", "csp_jog", "csv_velocity"]
LINKAGE_OPTIONS = [
    "independent",
    "left_right_mirror",
    "front_rear_sync",
    "side_pair",
    "diagonal_pair",
]


def ensure_absolute_name(name: str) -> str:
    if not name:
        return ""
    return name if name.startswith("/") else "/" + name


class FlipperMotorDebugUi:
    def __init__(self, flipper_ns: str, hybrid_ns: str):
        self.flipper_ns = ensure_absolute_name(flipper_ns)
        self.hybrid_ns = ensure_absolute_name(hybrid_ns)
        self.command_topic = self.flipper_ns + "/command"
        self.jog_topic = self.flipper_ns + "/jog_cmd"
        self.profile_service = self.flipper_ns + "/set_control_profile"
        self.linkage_service = self.flipper_ns + "/set_linkage_mode"
        self.state_topic = self.flipper_ns + "/state"
        self.runtime_state_topic = self.hybrid_ns + "/joint_runtime_states"

        joint_names = rospy.get_param(
            self.flipper_ns + "/joint_names", DEFAULT_JOINT_NAMES
        )
        if not isinstance(joint_names, list) or not joint_names:
            joint_names = list(DEFAULT_JOINT_NAMES)
        self.joint_names = [str(name) for name in joint_names]

        self.state_lock = threading.Lock()
        self.flipper_state = None
        self.runtime_states = {}
        self.status_text = "ready"

        self.trajectory_pub = rospy.Publisher(
            self.command_topic, JointTrajectory, queue_size=10
        )
        self.jog_pub = rospy.Publisher(self.jog_topic, JointJog, queue_size=10)
        self.profile_client = rospy.ServiceProxy(self.profile_service, SetControlProfile)
        self.linkage_client = rospy.ServiceProxy(self.linkage_service, SetLinkageMode)

        self.state_sub = rospy.Subscriber(
            self.state_topic, FlipperControlState, self.on_flipper_state, queue_size=1
        )
        self.runtime_sub = rospy.Subscriber(
            self.runtime_state_topic,
            JointRuntimeStateArray,
            self.on_runtime_state,
            queue_size=1,
        )

        self.root = tk.Tk()
        self.root.title("Flipper Motor Debug UI")
        self.root.geometry("1280x720")
        self.root.columnconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.profile_value_var = tk.StringVar(value=PROFILE_OPTIONS[0])
        self.linkage_value_var = tk.StringVar(value=LINKAGE_OPTIONS[0])
        self.trajectory_time_var = tk.StringVar(value="1.0")
        self.jog_duration_var = tk.StringVar(value="0.10")
        self.status_var = tk.StringVar(value=self.status_text)

        self.active_profile_var = tk.StringVar(value="-")
        self.active_hardware_mode_var = tk.StringVar(value="-")
        self.active_controller_var = tk.StringVar(value="-")
        self.active_linkage_var = tk.StringVar(value="-")
        self.switch_state_var = tk.StringVar(value="-")
        self.lifecycle_var = tk.StringVar(value="-")
        self.ready_var = tk.StringVar(value="-")
        self.switching_var = tk.StringVar(value="-")
        self.timeout_var = tk.StringVar(value="-")
        self.degraded_var = tk.StringVar(value="-")
        self.detail_var = tk.StringVar(value="-")

        self.position_entry_vars = {}
        self.velocity_entry_vars = {}
        self.measured_vars = {}
        self.reference_vars = {}
        self.commanded_vel_vars = {}
        self.online_vars = {}
        self.enabled_vars = {}
        self.fault_vars = {}
        self.runtime_lifecycle_vars = {}

        self.build_ui()
        self.root.after(100, self.refresh_ui)

    def build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(main, text="Runtime Summary", padding=8)
        summary.grid(row=0, column=0, sticky="ew")
        for column in range(4):
            summary.columnconfigure(column, weight=1)

        summary_rows = [
            ("active_profile", self.active_profile_var),
            ("hardware_mode", self.active_hardware_mode_var),
            ("controller", self.active_controller_var),
            ("linkage_mode", self.active_linkage_var),
            ("switch_state", self.switch_state_var),
            ("lifecycle", self.lifecycle_var),
            ("ready", self.ready_var),
            ("switching", self.switching_var),
            ("timed_out", self.timeout_var),
            ("degraded", self.degraded_var),
        ]
        for index, (label, value_var) in enumerate(summary_rows):
            row = index // 4
            column = (index % 4) * 2
            ttk.Label(summary, text=label).grid(row=row, column=column, sticky="w")
            ttk.Label(summary, textvariable=value_var).grid(
                row=row, column=column + 1, padx=(4, 12), sticky="w"
            )

        ttk.Label(summary, text="detail").grid(row=3, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.detail_var).grid(
            row=3, column=1, columnspan=7, sticky="w"
        )

        control = ttk.LabelFrame(main, text="Mode Control", padding=8)
        control.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            control.columnconfigure(column, weight=1)

        ttk.Label(control, text="profile").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            control,
            textvariable=self.profile_value_var,
            values=PROFILE_OPTIONS,
            state="readonly",
            width=18,
        ).grid(row=0, column=1, sticky="w")
        ttk.Button(
            control,
            text="Switch Profile",
            command=self.on_switch_profile,
        ).grid(row=0, column=2, padx=(6, 18), sticky="w")

        ttk.Label(control, text="linkage").grid(row=0, column=3, sticky="w")
        ttk.Combobox(
            control,
            textvariable=self.linkage_value_var,
            values=LINKAGE_OPTIONS,
            state="readonly",
            width=18,
        ).grid(row=0, column=4, sticky="w")
        ttk.Button(
            control,
            text="Switch Linkage",
            command=self.on_switch_linkage,
        ).grid(row=0, column=5, padx=(6, 18), sticky="w")

        ttk.Label(control, text="traj_time").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(control, textvariable=self.trajectory_time_var, width=10).grid(
            row=1, column=1, sticky="w", pady=(8, 0)
        )
        ttk.Label(control, text="jog_duration").grid(
            row=1, column=3, sticky="w", pady=(8, 0)
        )
        ttk.Entry(control, textvariable=self.jog_duration_var, width=10).grid(
            row=1, column=4, sticky="w", pady=(8, 0)
        )

        joints = ttk.LabelFrame(main, text="Joint Commands", padding=8)
        joints.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for column in range(10):
            joints.columnconfigure(column, weight=1)

        headers = [
            "joint",
            "online",
            "enabled",
            "fault",
            "measured",
            "reference",
            "cmd_vel",
            "lifecycle",
            "target_pos",
            "target_vel",
        ]
        for column, header in enumerate(headers):
            ttk.Label(joints, text=header).grid(
                row=0, column=column, padx=4, pady=(0, 6), sticky="w"
            )

        for row, joint_name in enumerate(self.joint_names, start=1):
            self.measured_vars[joint_name] = tk.StringVar(value="-")
            self.reference_vars[joint_name] = tk.StringVar(value="-")
            self.commanded_vel_vars[joint_name] = tk.StringVar(value="-")
            self.online_vars[joint_name] = tk.StringVar(value="-")
            self.enabled_vars[joint_name] = tk.StringVar(value="-")
            self.fault_vars[joint_name] = tk.StringVar(value="-")
            self.runtime_lifecycle_vars[joint_name] = tk.StringVar(value="-")
            self.position_entry_vars[joint_name] = tk.StringVar(value="")
            self.velocity_entry_vars[joint_name] = tk.StringVar(value="")

            ttk.Label(joints, text=joint_name).grid(row=row, column=0, sticky="w", padx=4)
            ttk.Label(joints, textvariable=self.online_vars[joint_name]).grid(
                row=row, column=1, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.enabled_vars[joint_name]).grid(
                row=row, column=2, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.fault_vars[joint_name]).grid(
                row=row, column=3, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.measured_vars[joint_name]).grid(
                row=row, column=4, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.reference_vars[joint_name]).grid(
                row=row, column=5, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.commanded_vel_vars[joint_name]).grid(
                row=row, column=6, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.runtime_lifecycle_vars[joint_name]).grid(
                row=row, column=7, sticky="w", padx=4
            )
            ttk.Entry(
                joints,
                textvariable=self.position_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=8, sticky="ew", padx=4)
            ttk.Entry(
                joints,
                textvariable=self.velocity_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=9, sticky="ew", padx=4)

        actions = ttk.Frame(main, padding=(0, 10, 0, 0))
        actions.grid(row=3, column=0, sticky="ew")
        ttk.Button(actions, text="Send Position Trajectory", command=self.on_send_trajectory).grid(
            row=0, column=0, padx=(0, 8), sticky="w"
        )
        ttk.Button(actions, text="Send Velocity Jog", command=self.on_send_jog).grid(
            row=0, column=1, padx=(0, 8), sticky="w"
        )
        ttk.Label(actions, textvariable=self.status_var).grid(row=0, column=2, sticky="w")

    def on_close(self) -> None:
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()

    def set_status(self, text: str) -> None:
        self.status_text = text
        self.status_var.set(text)

    def call_service_async(self, func, success_text: str) -> None:
        def worker() -> None:
            try:
                func()
                self.root.after(0, lambda: self.set_status(success_text))
            except rospy.ServiceException as exc:
                self.root.after(
                    0,
                    lambda: messagebox.showerror("Service Error", str(exc)),
                )
                self.root.after(0, lambda: self.set_status("service failed"))

        threading.Thread(target=worker, daemon=True).start()

    def on_switch_profile(self) -> None:
        profile = self.profile_value_var.get()

        def request() -> None:
            self.profile_client.wait_for_service(timeout=2.0)
            self.profile_client(profile)

        self.call_service_async(request, "profile switched")

    def on_switch_linkage(self) -> None:
        linkage = self.linkage_value_var.get()

        def request() -> None:
            self.linkage_client.wait_for_service(timeout=2.0)
            self.linkage_client(linkage)

        self.call_service_async(request, "linkage switched")

    def parse_duration(self, value: str, label: str) -> float:
        try:
            duration = float(value)
        except ValueError as exc:
            raise ValueError(f"{label} must be a number") from exc
        if duration <= 0.0:
            raise ValueError(f"{label} must be > 0")
        return duration

    def on_send_trajectory(self) -> None:
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
            names = []
            positions = []
            for joint_name in self.joint_names:
                raw = self.position_entry_vars[joint_name].get().strip()
                if not raw:
                    continue
                names.append(joint_name)
                positions.append(float(raw))
            if not names:
                raise ValueError("no target_pos values provided")
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            return

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration.from_sec(duration)
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.set_status("position trajectory sent")

    def on_send_jog(self) -> None:
        try:
            duration = self.parse_duration(self.jog_duration_var.get(), "jog_duration")
            names = []
            velocities = []
            for joint_name in self.joint_names:
                raw = self.velocity_entry_vars[joint_name].get().strip()
                if not raw:
                    continue
                names.append(joint_name)
                velocities.append(float(raw))
            if not names:
                raise ValueError("no target_vel values provided")
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            return

        msg = JointJog()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = names
        msg.velocities = velocities
        msg.duration = duration
        self.jog_pub.publish(msg)
        self.set_status("velocity jog sent")

    def on_flipper_state(self, msg: FlipperControlState) -> None:
        with self.state_lock:
            self.flipper_state = msg

    def on_runtime_state(self, msg: JointRuntimeStateArray) -> None:
        updated = {}
        for state in msg.states:
            updated[state.joint_name] = state
        with self.state_lock:
            self.runtime_states = updated

    @staticmethod
    def format_number(value: float) -> str:
        return f"{value:.4f}"

    def refresh_ui(self) -> None:
        with self.state_lock:
            flipper_state = self.flipper_state
            runtime_states = dict(self.runtime_states)

        if flipper_state is not None:
            self.active_profile_var.set(flipper_state.active_profile)
            self.active_hardware_mode_var.set(flipper_state.active_hardware_mode)
            self.active_controller_var.set(flipper_state.active_controller)
            self.active_linkage_var.set(flipper_state.active_linkage_mode)
            self.switch_state_var.set(flipper_state.switch_state)
            self.lifecycle_var.set(flipper_state.lifecycle_state)
            self.ready_var.set(str(flipper_state.ready))
            self.switching_var.set(str(flipper_state.switching))
            self.timeout_var.set(str(flipper_state.command_stream_timed_out))
            self.degraded_var.set(str(flipper_state.degraded))
            self.detail_var.set(flipper_state.detail or "-")

            measured = dict(zip(flipper_state.joint_names, flipper_state.measured_positions))
            reference = dict(
                zip(flipper_state.joint_names, flipper_state.reference_positions)
            )
            commanded_vel = dict(
                zip(flipper_state.joint_names, flipper_state.commanded_velocities)
            )
            for joint_name in self.joint_names:
                if joint_name in measured:
                    self.measured_vars[joint_name].set(
                        self.format_number(measured[joint_name])
                    )
                if joint_name in reference:
                    self.reference_vars[joint_name].set(
                        self.format_number(reference[joint_name])
                    )
                if joint_name in commanded_vel:
                    self.commanded_vel_vars[joint_name].set(
                        self.format_number(commanded_vel[joint_name])
                    )

        for joint_name in self.joint_names:
            runtime = runtime_states.get(joint_name)
            if runtime is None:
                self.online_vars[joint_name].set("-")
                self.enabled_vars[joint_name].set("-")
                self.fault_vars[joint_name].set("-")
                self.runtime_lifecycle_vars[joint_name].set("-")
                continue
            self.online_vars[joint_name].set(str(runtime.online))
            self.enabled_vars[joint_name].set(str(runtime.enabled))
            self.fault_vars[joint_name].set(str(runtime.fault))
            self.runtime_lifecycle_vars[joint_name].set(runtime.lifecycle_state)

        self.root.after(200, self.refresh_ui)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flipper motor debug UI")
    parser.add_argument(
        "--flipper-ns",
        default=DEFAULT_FLIPPER_NS,
        help="Namespace for flipper_control topics and services.",
    )
    parser.add_argument(
        "--hybrid-ns",
        default=DEFAULT_HYBRID_NS,
        help="Namespace for hybrid runtime topics.",
    )
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()
    rospy.init_node("flipper_motor_debug_ui", anonymous=True)
    ui = FlipperMotorDebugUi(args.flipper_ns, args.hybrid_ns)
    ui.run()


if __name__ == "__main__":
    main()
