#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rospy
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from Eyou_ROS1_Master.msg import JointRuntimeStateArray


DEFAULT_HYBRID_NS = "/hybrid_motor_hw_node"
DEFAULT_CONTROLLER_MANAGER_NS = "/controller_manager"
DEFAULT_COMMAND_TOPIC = "/arm_rear4_position_controller/command"
DEFAULT_GRIPPER_COMMAND_TOPIC = ""
DEFAULT_RUNTIME_STATE_TOPIC = "/hybrid_motor_hw_node/joint_runtime_states"
DEFAULT_CONTROLLER_NAME = "arm_rear4_position_controller"
DEFAULT_GRIPPER_CONTROLLER_NAME = ""
DEFAULT_GRIPPER_CLOSED_REFERENCE_POSITION = 4.808646
DEFAULT_GRIPPER_STROKE = 0.06
DEFAULT_JOINT_NAMES = [
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
]
DEFAULT_GRIPPER_JOINT_NAMES = []
LIFECYCLE_SERVICES = (
    "init",
    "enable",
    "disable",
    "halt",
    "resume",
    "recover",
    "shutdown",
)
SERVICE_SUCCESS_STATES = {
    "init": "Armed",
    "enable": "Armed",
    "disable": "Standby",
    "halt": "Armed",
    "resume": "Running",
    "recover": "Standby",
    "shutdown": "Configured",
}
SERVICE_MESSAGE_STATES = (
    ("already running", "Running"),
    ("already enabled", "Armed"),
    ("already initialized", "Armed"),
    ("already halted", "Armed"),
    ("already disabled", "Standby"),
    ("initialized (armed)", "Armed"),
    ("enabled (armed)", "Armed"),
    ("disabled (standby)", "Standby"),
    ("recovered (standby)", "Standby"),
    ("communication stopped", "Configured"),
)
CANNOT_TRANSITION_RE = re.compile(r"cannot transition from ([A-Za-z]+) to", re.I)


def ensure_absolute_name(name: str) -> str:
    normalized = str(name or "").strip()
    if not normalized:
        return ""
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    if len(normalized) > 1:
        normalized = normalized.rstrip("/")
    return normalized


def canonical_lifecycle_name(value: str) -> str:
    normalized = re.sub(r"[^a-z]", "", str(value or "").strip().lower())
    return {
        "inactive": "Inactive",
        "configured": "Configured",
        "standby": "Standby",
        "armed": "Armed",
        "running": "Running",
        "faulted": "Faulted",
        "recovering": "Recovering",
        "shuttingdown": "ShuttingDown",
        "unknown": "Unknown",
    }.get(normalized, str(value or "").strip() or "Unknown")


class ArmRear4DebugUi:
    def __init__(self, hybrid_ns: str, controller_manager_ns: str):
        self.hybrid_ns = ensure_absolute_name(hybrid_ns)
        self.controller_manager_ns = ensure_absolute_name(controller_manager_ns)
        self.command_topic = ensure_absolute_name(
            rospy.get_param("~command_topic", DEFAULT_COMMAND_TOPIC)
        )
        self.gripper_command_topic = ensure_absolute_name(
            rospy.get_param("~gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        )
        self.runtime_state_topic = ensure_absolute_name(
            rospy.get_param("~runtime_state_topic", DEFAULT_RUNTIME_STATE_TOPIC)
        )
        self.controller_name = str(
            rospy.get_param("~controller_name", DEFAULT_CONTROLLER_NAME)
        )
        self.gripper_controller_name = str(
            rospy.get_param("~gripper_controller_name", DEFAULT_GRIPPER_CONTROLLER_NAME)
        )
        self.gripper_closed_reference_position = float(
            rospy.get_param(
                "~gripper_closed_reference_position",
                DEFAULT_GRIPPER_CLOSED_REFERENCE_POSITION,
            )
        )
        self.gripper_stroke = float(
            rospy.get_param("~gripper_stroke", DEFAULT_GRIPPER_STROKE)
        )
        self.gripper_target_mode = str(
            rospy.get_param("~gripper_target_mode", "absolute")
        ).strip().lower()
        joint_names = rospy.get_param("~joint_names", list(DEFAULT_JOINT_NAMES))
        if not isinstance(joint_names, list) or not joint_names:
            joint_names = list(DEFAULT_JOINT_NAMES)
        gripper_joint_names = rospy.get_param(
            "~gripper_joint_names", list(DEFAULT_GRIPPER_JOINT_NAMES)
        )
        if not isinstance(gripper_joint_names, list):
            gripper_joint_names = list(DEFAULT_GRIPPER_JOINT_NAMES)
        self.arm_joint_names = [str(name) for name in joint_names]
        self.gripper_joint_names = [str(name) for name in gripper_joint_names]
        self.joint_names = self.arm_joint_names + self.gripper_joint_names
        self.window_title = str(rospy.get_param("~window_title", "Arm Rear4 Debug UI"))
        self.joint_group_title = str(rospy.get_param("~joint_group_title", "Rear4 Joints"))

        self.state_lock = threading.Lock()
        self.runtime_states = {}
        self.controller_states = {}
        self.status_text = "ready"
        self.lifecycle_estimate = ""
        self.lifecycle_source = "unknown"
        self.controller_poll_inflight = False
        self.last_controller_poll_monotonic = 0.0
        self.controller_poll_interval_sec = 1.0
        self.closed = False

        self.trajectory_pub = rospy.Publisher(
            self.command_topic, JointTrajectory, queue_size=10
        )
        self.gripper_trajectory_pub = None
        if self.gripper_command_topic and self.gripper_joint_names:
            self.gripper_trajectory_pub = rospy.Publisher(
                self.gripper_command_topic, JointTrajectory, queue_size=10
            )
        self.runtime_sub = rospy.Subscriber(
            self.runtime_state_topic,
            JointRuntimeStateArray,
            self.on_runtime_state,
            queue_size=1,
        )

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("1280x760")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.trajectory_time_var = tk.StringVar(value="1.0")
        self.step_delta_var = tk.StringVar(value="0.05")
        self.status_var = tk.StringVar(value=self.status_text)
        self.hybrid_ns_var = tk.StringVar(value=self.hybrid_ns)
        self.command_topic_var = tk.StringVar(value=self.command_topic)
        self.gripper_command_topic_var = tk.StringVar(value=self.gripper_command_topic or "-")
        self.runtime_topic_var = tk.StringVar(value=self.runtime_state_topic)
        self.controller_var = tk.StringVar(value=self.controller_name)
        self.gripper_controller_var = tk.StringVar(value=self.gripper_controller_name or "-")
        self.lifecycle_var = tk.StringVar(value="-")
        self.lifecycle_source_var = tk.StringVar(value="-")
        self.backend_detail_var = tk.StringVar(value="-")
        self.controller_state_var = tk.StringVar(value="-")
        self.gripper_controller_state_var = tk.StringVar(value="-")
        self.joint_state_controller_var = tk.StringVar(value="-")

        self.position_entry_vars = {}
        self.measured_vars = {}
        self.reference_vars = {}
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
        main.rowconfigure(3, weight=1)

        summary = ttk.LabelFrame(main, text="Runtime Summary", padding=8)
        summary.grid(row=0, column=0, sticky="ew")
        for column in range(4):
            summary.columnconfigure(column, weight=1)

        summary_rows = [
            ("hybrid_ns", self.hybrid_ns_var),
            ("command_topic", self.command_topic_var),
            ("gripper_topic", self.gripper_command_topic_var),
            ("runtime_topic", self.runtime_topic_var),
            ("controller", self.controller_var),
            ("gripper_ctrl", self.gripper_controller_var),
            ("lifecycle", self.lifecycle_var),
            ("lifecycle_src", self.lifecycle_source_var),
            ("joint_state_ctrl", self.joint_state_controller_var),
            ("arm_ctrl", self.controller_state_var),
            ("gripper_ctrl", self.gripper_controller_state_var),
        ]
        for index, (label, var) in enumerate(summary_rows):
            row = index // 2
            column = (index % 2) * 2
            ttk.Label(summary, text=label).grid(row=row, column=column, sticky="w")
            ttk.Label(summary, textvariable=var).grid(
                row=row, column=column + 1, sticky="w", padx=(4, 12)
            )

        ttk.Label(summary, text="detail").grid(row=6, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.backend_detail_var).grid(
            row=6, column=1, columnspan=3, sticky="w"
        )

        backend = ttk.LabelFrame(main, text="Lifecycle Ops", padding=8)
        backend.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            backend.columnconfigure(column, weight=1)

        ttk.Button(
            backend,
            text="Refresh Controllers",
            command=self.refresh_controller_states,
        ).grid(row=0, column=0, padx=(0, 8), sticky="w")

        for index, service_name in enumerate(LIFECYCLE_SERVICES, start=1):
            ttk.Button(
                backend,
                text=service_name,
                command=lambda name=service_name: self.call_lifecycle_service(name),
            ).grid(row=0, column=index, padx=2, sticky="w")

        control = ttk.LabelFrame(main, text="Motion Control", padding=8)
        control.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            control.columnconfigure(column, weight=1)

        ttk.Label(control, text="traj_time").grid(row=0, column=0, sticky="w")
        ttk.Entry(control, textvariable=self.trajectory_time_var, width=10).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(control, text="step_delta").grid(row=0, column=2, sticky="w")
        ttk.Entry(control, textvariable=self.step_delta_var, width=10).grid(
            row=0, column=3, sticky="w"
        )
        ttk.Button(
            control,
            text="Send Trajectory",
            command=self.on_send_trajectory,
        ).grid(row=0, column=4, padx=(8, 0), sticky="w")
        ttk.Button(
            control,
            text="Stop All",
            command=self.on_stop_all,
        ).grid(row=0, column=5, padx=(8, 0), sticky="w")

        joints = ttk.LabelFrame(main, text=self.joint_group_title, padding=8)
        joints.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        for column in range(10):
            joints.columnconfigure(column, weight=1)

        headers = [
            "joint",
            "online",
            "enabled",
            "fault",
            "measured",
            "reference",
            "lifecycle",
            "target",
            "-step",
            "+step",
        ]
        for column, header in enumerate(headers):
            ttk.Label(joints, text=header).grid(
                row=0, column=column, padx=4, pady=(0, 6), sticky="w"
            )

        for row, joint_name in enumerate(self.joint_names, start=1):
            self.measured_vars[joint_name] = tk.StringVar(value="-")
            self.reference_vars[joint_name] = tk.StringVar(value="-")
            self.online_vars[joint_name] = tk.StringVar(value="-")
            self.enabled_vars[joint_name] = tk.StringVar(value="-")
            self.fault_vars[joint_name] = tk.StringVar(value="-")
            self.runtime_lifecycle_vars[joint_name] = tk.StringVar(value="-")
            self.position_entry_vars[joint_name] = tk.StringVar(value="")

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
            ttk.Label(joints, textvariable=self.runtime_lifecycle_vars[joint_name]).grid(
                row=row, column=6, sticky="w", padx=4
            )
            ttk.Entry(
                joints,
                textvariable=self.position_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=7, sticky="ew", padx=4)
            ttk.Button(
                joints,
                text="-",
                command=lambda name=joint_name: self.on_step_joint(name, -1.0),
            ).grid(row=row, column=8, sticky="w", padx=4)
            ttk.Button(
                joints,
                text="+",
                command=lambda name=joint_name: self.on_step_joint(name, 1.0),
            ).grid(row=row, column=9, sticky="w", padx=4)

        actions = ttk.Frame(main, padding=(0, 10, 0, 0))
        actions.grid(row=4, column=0, sticky="ew")
        ttk.Label(actions, textvariable=self.status_var).grid(row=0, column=0, sticky="w")

    def run(self) -> None:
        self.root.mainloop()

    def on_close(self) -> None:
        self.closed = True
        try:
            self.runtime_sub.unregister()
        except Exception:
            pass
        rospy.signal_shutdown("ui closed")
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def set_status(self, text: str) -> None:
        with self.state_lock:
            self.status_text = text

    def apply_lifecycle_observation(self, state: str, source: str) -> None:
        if not state or state == "Unknown":
            return
        with self.state_lock:
            self.lifecycle_estimate = state
            self.lifecycle_source = source

    def infer_lifecycle_state_from_service_result(self, service_name: str, success: bool, message: str):
        if success:
            return SERVICE_SUCCESS_STATES.get(service_name)
        lower = message.lower()
        for pattern, state in SERVICE_MESSAGE_STATES:
            if pattern in lower:
                return state
        match = CANNOT_TRANSITION_RE.search(message)
        if match:
            return canonical_lifecycle_name(match.group(1))
        if "faulted" in lower:
            return "Faulted"
        if "recovering" in lower:
            return "Recovering"
        return None

    def lifecycle_service_name(self, service_name: str) -> str:
        return self.hybrid_ns + "/" + service_name

    def call_lifecycle_service(self, service_name: str) -> None:
        self.set_status(f"{service_name}: calling {self.lifecycle_service_name(service_name)} ...")
        threading.Thread(
            target=self._call_lifecycle_service_worker,
            args=(service_name,),
            daemon=True,
        ).start()

    def _call_lifecycle_service_worker(self, service_name: str) -> None:
        full_name = self.lifecycle_service_name(service_name)
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, Trigger)
            response = proxy()
            lifecycle = self.infer_lifecycle_state_from_service_result(
                service_name, bool(response.success), response.message
            )
            if lifecycle is not None:
                self.apply_lifecycle_observation(lifecycle, "service")
            prefix = "OK" if response.success else "FAIL"
            self.set_status(f"{service_name}: {prefix} {response.message}")
        except Exception as exc:
            self.set_status(f"{service_name}: ERROR {exc}")

    def refresh_controller_states(self) -> None:
        self._maybe_poll_controller_states(force=True)

    def _maybe_poll_controller_states(self, force: bool = False) -> None:
        now = time.monotonic()
        with self.state_lock:
            if self.controller_poll_inflight:
                return
            if not force and (now - self.last_controller_poll_monotonic) < self.controller_poll_interval_sec:
                return
            self.controller_poll_inflight = True
            self.last_controller_poll_monotonic = now
        threading.Thread(target=self._poll_controller_states_worker, daemon=True).start()

    def _poll_controller_states_worker(self) -> None:
        try:
            full_name = self.controller_manager_ns + "/list_controllers"
            rospy.wait_for_service(full_name, timeout=1.0)
            proxy = rospy.ServiceProxy(full_name, ListControllers)
            response = proxy()
            updated = {item.name: item.state for item in response.controller}
            with self.state_lock:
                self.controller_states = updated
        except Exception:
            pass
        finally:
            with self.state_lock:
                self.controller_poll_inflight = False

    def parse_duration(self, value: str, label: str) -> float:
        try:
            duration = float(value)
        except ValueError as exc:
            raise ValueError(f"{label} must be a number") from exc
        if duration <= 0.0:
            raise ValueError(f"{label} must be > 0")
        return duration

    def parse_step_delta(self) -> float:
        try:
            delta = float(self.step_delta_var.get())
        except ValueError as exc:
            raise ValueError("step_delta must be a number") from exc
        if delta <= 0.0:
            raise ValueError("step_delta must be > 0")
        return delta

    def publish_positions(self, joint_targets, duration: float) -> None:
        arm_targets = self.complete_group_targets(
            self.arm_joint_names,
            {
                name: joint_targets[name]
                for name in self.arm_joint_names
                if name in joint_targets
            },
        )
        gripper_targets = self.complete_group_targets(
            self.gripper_joint_names,
            {
                name: joint_targets[name]
                for name in self.gripper_joint_names
                if name in joint_targets
            },
        )
        self.publish_group_positions(self.trajectory_pub, arm_targets, duration)
        if gripper_targets:
            if self.gripper_trajectory_pub is None:
                raise ValueError("gripper command topic is not configured")
            gripper_targets = {
                name: self.gripper_target_to_absolute(value)
                for name, value in gripper_targets.items()
            }
            self.publish_group_positions(
                self.gripper_trajectory_pub, gripper_targets, duration
            )

    def complete_group_targets(self, group_joint_names, provided_targets):
        if not provided_targets:
            return {}
        with self.state_lock:
            runtime_states = dict(self.runtime_states)
        completed = {}
        for joint_name in group_joint_names:
            if joint_name in provided_targets:
                completed[joint_name] = provided_targets[joint_name]
                continue

            raw_entry = self.position_entry_vars.get(joint_name)
            if raw_entry is not None:
                raw = raw_entry.get().strip()
                if raw:
                    completed[joint_name] = float(raw)
                    continue

            runtime = runtime_states.get(joint_name)
            if runtime is None:
                raise ValueError(
                    f"{joint_name} has no target/runtime state; cannot build full trajectory"
                )
            reference_position = float(getattr(runtime, "reference_position", 0.0))
            if joint_name in self.gripper_joint_names:
                reference_position = self.gripper_absolute_to_display(reference_position)
            completed[joint_name] = reference_position
        return completed

    def gripper_target_to_absolute(self, value: float) -> float:
        if self.gripper_target_mode != "opening":
            return value
        opening = max(0.0, min(self.gripper_stroke, value))
        return self.gripper_closed_reference_position - opening

    def gripper_absolute_to_display(self, value: float) -> float:
        if self.gripper_target_mode != "opening":
            return value
        opening = self.gripper_closed_reference_position - value
        return max(0.0, min(self.gripper_stroke, opening))

    @staticmethod
    def publish_group_positions(publisher, joint_targets, duration: float) -> None:
        if not joint_targets:
            return
        if publisher.get_num_connections() <= 0:
            topic_name = getattr(
                publisher,
                "resolved_name",
                getattr(publisher, "name", "trajectory topic"),
            )
            raise ValueError(f"{topic_name} has no subscribers")
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = list(joint_targets.keys())
        point = JointTrajectoryPoint()
        point.positions = [joint_targets[name] for name in traj.joint_names]
        point.time_from_start = rospy.Duration.from_sec(duration)
        traj.points = [point]
        publisher.publish(traj)

    def on_send_trajectory(self) -> None:
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
            joint_targets = {}
            for joint_name in self.joint_names:
                raw = self.position_entry_vars[joint_name].get().strip()
                if not raw:
                    continue
                joint_targets[joint_name] = float(raw)
            if not joint_targets:
                raise ValueError("no target_pos values provided")
        except ValueError as exc:
            messagebox.showerror("Invalid Input", str(exc))
            return

        try:
            self.publish_positions(joint_targets, duration)
        except ValueError as exc:
            messagebox.showerror("Send Trajectory Failed", str(exc))
            self.set_status(f"send trajectory failed: {exc}")
            return
        self.set_status("position trajectory sent")

    def on_step_joint(self, joint_name: str, direction: float) -> None:
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
            delta = self.parse_step_delta()
            with self.state_lock:
                runtime = self.runtime_states.get(joint_name)
            if runtime is None:
                raise ValueError(f"{joint_name} has no runtime state yet")
            base = float(getattr(runtime, "reference_position", 0.0))
            if joint_name in self.gripper_joint_names:
                base = self.gripper_absolute_to_display(base)
            target = base + direction * delta
            self.position_entry_vars[joint_name].set(f"{target:.4f}")
            self.publish_positions({joint_name: target}, duration)
        except ValueError as exc:
            messagebox.showerror("Invalid Step", str(exc))
            self.set_status(f"step failed for {joint_name}: {exc}")
            return
        self.set_status(f"step sent for {joint_name}: {target:.4f}")

    def on_stop_all(self) -> None:
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
            with self.state_lock:
                snapshot = dict(self.runtime_states)
            joint_targets = {}
            for joint_name in self.joint_names:
                runtime = snapshot.get(joint_name)
                if runtime is None:
                    continue
                measured_position = float(getattr(runtime, "measured_position", 0.0))
                if joint_name in self.gripper_joint_names:
                    measured_position = self.gripper_absolute_to_display(measured_position)
                joint_targets[joint_name] = measured_position
            if not joint_targets:
                raise ValueError("no runtime joint state available")
        except ValueError as exc:
            messagebox.showerror("Stop All Failed", str(exc))
            return

        try:
            self.publish_positions(joint_targets, duration)
        except ValueError as exc:
            messagebox.showerror("Stop All Failed", str(exc))
            self.set_status(f"stop-all failed: {exc}")
            return
        self.set_status("stop-all hold trajectory sent")

    def on_runtime_state(self, msg: JointRuntimeStateArray) -> None:
        updated = {}
        first_lifecycle = ""
        for state in msg.states:
            if state.joint_name not in self.joint_names:
                continue
            updated[state.joint_name] = state
            if not first_lifecycle and state.lifecycle_state:
                first_lifecycle = canonical_lifecycle_name(state.lifecycle_state)
        with self.state_lock:
            merged = dict(self.runtime_states)
            merged.update(updated)
            self.runtime_states = merged
        if first_lifecycle and first_lifecycle != "Unknown":
            self.apply_lifecycle_observation(first_lifecycle, "runtime")

    @staticmethod
    def format_number(value: float) -> str:
        return f"{value:.4f}"

    def summarize_runtime_states(self, runtime_states) -> str:
        if not runtime_states:
            return "waiting for joint_runtime_states"
        fault_joints = sorted(
            joint_name
            for joint_name, state in runtime_states.items()
            if getattr(state, "fault", False)
        )
        offline_joints = sorted(
            joint_name
            for joint_name, state in runtime_states.items()
            if not getattr(state, "online", False)
        )
        if fault_joints:
            return "fault: " + ", ".join(fault_joints)
        if offline_joints:
            return "offline: " + ", ".join(offline_joints)
        return "runtime states OK"

    def resolve_lifecycle(self, runtime_states, estimate: str, estimate_source: str):
        lifecycles = sorted(
            {
                canonical_lifecycle_name(getattr(state, "lifecycle_state", ""))
                for state in runtime_states.values()
                if getattr(state, "lifecycle_state", "")
            }
        )
        lifecycles = [state for state in lifecycles if state != "Unknown"]
        detail = self.summarize_runtime_states(runtime_states)
        if len(lifecycles) == 1:
            return lifecycles[0], "runtime", detail
        if len(lifecycles) > 1:
            return "Mixed", "runtime", detail
        if estimate:
            return estimate, estimate_source, detail
        return "-", "-", detail

    def refresh_ui(self) -> None:
        self._maybe_poll_controller_states()
        with self.state_lock:
            runtime_states = dict(self.runtime_states)
            controller_states = dict(self.controller_states)
            status_text = self.status_text
            estimate = self.lifecycle_estimate
            estimate_source = self.lifecycle_source

        lifecycle, lifecycle_source, detail = self.resolve_lifecycle(
            runtime_states, estimate, estimate_source
        )
        self.status_var.set(status_text)
        self.lifecycle_var.set(lifecycle)
        self.lifecycle_source_var.set(lifecycle_source)
        self.backend_detail_var.set(detail)
        self.controller_state_var.set(controller_states.get(self.controller_name, "-"))
        if self.gripper_controller_name:
            self.gripper_controller_state_var.set(
                controller_states.get(self.gripper_controller_name, "-")
            )
        else:
            self.gripper_controller_state_var.set("-")
        self.joint_state_controller_var.set(
            controller_states.get("joint_state_controller", "-")
        )

        for joint_name in self.joint_names:
            runtime = runtime_states.get(joint_name)
            if runtime is None:
                self.online_vars[joint_name].set("-")
                self.enabled_vars[joint_name].set("-")
                self.fault_vars[joint_name].set("-")
                self.measured_vars[joint_name].set("-")
                self.reference_vars[joint_name].set("-")
                self.runtime_lifecycle_vars[joint_name].set("-")
                continue

            self.online_vars[joint_name].set("1" if runtime.online else "0")
            self.enabled_vars[joint_name].set("1" if runtime.enabled else "0")
            self.fault_vars[joint_name].set("1" if runtime.fault else "0")
            self.measured_vars[joint_name].set(
                self.format_number(
                    self.gripper_absolute_to_display(runtime.measured_position)
                    if joint_name in self.gripper_joint_names
                    else runtime.measured_position
                )
            )
            self.reference_vars[joint_name].set(
                self.format_number(
                    self.gripper_absolute_to_display(runtime.reference_position)
                    if joint_name in self.gripper_joint_names
                    else runtime.reference_position
                )
            )
            self.runtime_lifecycle_vars[joint_name].set(
                canonical_lifecycle_name(runtime.lifecycle_state)
            )

        if not self.closed and not rospy.is_shutdown():
            self.root.after(200, self.refresh_ui)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Arm rear4 debug UI")
    parser.add_argument(
        "--hybrid-ns",
        default=DEFAULT_HYBRID_NS,
        help="Namespace for hybrid backend services and runtime topics.",
    )
    parser.add_argument(
        "--controller-manager-ns",
        default=DEFAULT_CONTROLLER_MANAGER_NS,
        help="Namespace for controller_manager services.",
    )
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()
    rospy.init_node("arm_rear4_debug_ui", anonymous=False, disable_signals=True)
    hybrid_ns = rospy.get_param("~hybrid_ns", args.hybrid_ns)
    controller_manager_ns = rospy.get_param(
        "~controller_manager_ns", args.controller_manager_ns
    )
    ui = ArmRear4DebugUi(hybrid_ns, controller_manager_ns)
    ui.run()


if __name__ == "__main__":
    main()