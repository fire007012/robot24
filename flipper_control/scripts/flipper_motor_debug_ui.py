#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Dict, Optional

import rospy
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointJog
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from Eyou_ROS1_Master.msg import JointRuntimeStateArray
from flipper_control.msg import FlipperControlState
from flipper_control.srv import SetControlProfile, SetLinkageMode


DEFAULT_FLIPPER_NS = "/flipper_control"
DEFAULT_HYBRID_NS = "/hybrid_motor_hw_node"
DEFAULT_CANOPEN_NS = "/canopen_hw_node"
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
    if not name:
        return ""
    normalized = name.strip()
    if not normalized:
        return ""
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    if len(normalized) > 1:
        normalized = normalized.rstrip("/")
    return normalized


def parse_boolish(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def level_text(level: int) -> str:
    return {
        0: "OK",
        1: "WARN",
        2: "ERROR",
        3: "STALE",
    }.get(level, f"L{level}")


def canonical_lifecycle_name(value: str) -> str:
    normalized = re.sub(r"[^a-z]", "", value.strip().lower())
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
    }.get(normalized, value.strip() or "Unknown")


@dataclass
class CanopenDiagnosticState:
    summary: str = "STALE:no data"
    operational: bool = False
    fault: bool = False
    heartbeat_lost: bool = False


class FlipperMotorDebugUi:
    def __init__(
        self,
        flipper_ns: str,
        hybrid_ns: str,
        canopen_ns: str,
        backend_type_override: str,
    ):
        self.flipper_ns = ensure_absolute_name(flipper_ns)
        self.hybrid_ns = ensure_absolute_name(hybrid_ns)
        self.canopen_ns = ensure_absolute_name(canopen_ns)

        configured_backend = str(
            rospy.get_param(self.flipper_ns + "/backend_type", "hybrid")
        ).strip().lower()
        requested_backend = backend_type_override.strip().lower()
        if requested_backend in {"hybrid", "canopen"}:
            self.backend_type = requested_backend
        elif configured_backend in {"hybrid", "canopen"}:
            self.backend_type = configured_backend
        else:
            self.backend_type = "hybrid"

        self.command_topic = self.flipper_ns + "/command"
        self.jog_topic = self.flipper_ns + "/jog_cmd"
        self.profile_service = self.flipper_ns + "/set_control_profile"
        self.linkage_service = self.flipper_ns + "/set_linkage_mode"
        self.state_topic = self.flipper_ns + "/state"
        self.runtime_state_topic = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/runtime_state_topic",
                    self.hybrid_ns + "/joint_runtime_states",
                )
            )
        )
        self.canopen_diagnostics_topic = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/canopen_diagnostics_topic",
                    "/diagnostics",
                )
            )
        )
        self.controller_manager_ns = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/controller_manager_ns",
                    "/controller_manager",
                )
            )
        )
        self.backend_service_ns = (
            self.hybrid_ns if self.backend_type == "hybrid" else self.canopen_ns
        )

        joint_names = rospy.get_param(
            self.flipper_ns + "/joint_names", DEFAULT_JOINT_NAMES
        )
        if not isinstance(joint_names, list) or not joint_names:
            joint_names = list(DEFAULT_JOINT_NAMES)
        self.joint_names = [str(name) for name in joint_names]

        self.controllers = {
            "joint_state_controller": "joint_state_controller",
            "csp": str(
                rospy.get_param(
                    self.flipper_ns + "/controllers/csp", "flipper_csp_controller"
                )
            ),
            "csv": str(
                rospy.get_param(
                    self.flipper_ns + "/controllers/csv", "flipper_csv_controller"
                )
            ),
        }

        self.canopen_auto_init = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_init", False)
        )
        self.canopen_auto_enable = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_enable", False)
        )
        self.canopen_auto_release = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_release", False)
        )

        self.state_lock = threading.Lock()
        self.flipper_state: Optional[FlipperControlState] = None
        self.runtime_states: Dict[str, object] = {}
        self.canopen_diagnostics: Dict[str, CanopenDiagnosticState] = {}
        self.controller_states: Dict[str, str] = {}
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
        self.jog_pub = rospy.Publisher(self.jog_topic, JointJog, queue_size=10)

        self.state_sub = rospy.Subscriber(
            self.state_topic, FlipperControlState, self.on_flipper_state, queue_size=1
        )
        self.runtime_sub = None
        self.diagnostics_sub = None
        if self.backend_type == "hybrid":
            self.runtime_sub = rospy.Subscriber(
                self.runtime_state_topic,
                JointRuntimeStateArray,
                self.on_runtime_state,
                queue_size=1,
            )
        else:
            self.diagnostics_sub = rospy.Subscriber(
                self.canopen_diagnostics_topic,
                DiagnosticArray,
                self.on_diagnostics,
                queue_size=10,
            )

        self.root = tk.Tk()
        self.root.title("Flipper Motor Debug UI")
        self.root.geometry("1400x820")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.profile_value_var = tk.StringVar(value=PROFILE_OPTIONS[0])
        self.linkage_value_var = tk.StringVar(value=LINKAGE_OPTIONS[0])
        self.trajectory_time_var = tk.StringVar(value="1.0")
        self.jog_duration_var = tk.StringVar(value="0.10")
        self.status_var = tk.StringVar(value=self.status_text)

        self.backend_type_var = tk.StringVar(value=self.backend_type)
        self.backend_ns_var = tk.StringVar(value=self.backend_service_ns)
        self.active_profile_var = tk.StringVar(value="-")
        self.active_hardware_mode_var = tk.StringVar(value="-")
        self.active_controller_var = tk.StringVar(value="-")
        self.active_linkage_var = tk.StringVar(value="-")
        self.switch_state_var = tk.StringVar(value="-")
        self.lifecycle_var = tk.StringVar(value="-")
        self.lifecycle_source_var = tk.StringVar(value="-")
        self.ready_var = tk.StringVar(value="-")
        self.switching_var = tk.StringVar(value="-")
        self.timeout_var = tk.StringVar(value="-")
        self.degraded_var = tk.StringVar(value="-")
        self.detail_var = tk.StringVar(value="-")
        self.backend_detail_var = tk.StringVar(value="-")
        self.joint_state_controller_var = tk.StringVar(value="-")
        self.csp_controller_state_var = tk.StringVar(value="-")
        self.csv_controller_state_var = tk.StringVar(value="-")

        self.position_entry_vars = {}
        self.velocity_entry_vars = {}
        self.measured_vars = {}
        self.reference_vars = {}
        self.commanded_vel_vars = {}
        self.online_vars = {}
        self.enabled_vars = {}
        self.fault_vars = {}
        self.heartbeat_vars = {}
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
        for column in range(6):
            summary.columnconfigure(column, weight=1)

        summary_rows = [
            ("backend", self.backend_type_var),
            ("backend_ns", self.backend_ns_var),
            ("active_profile", self.active_profile_var),
            ("hardware_mode", self.active_hardware_mode_var),
            ("active_controller", self.active_controller_var),
            ("linkage_mode", self.active_linkage_var),
            ("switch_state", self.switch_state_var),
            ("lifecycle", self.lifecycle_var),
            ("lifecycle_src", self.lifecycle_source_var),
            ("ready", self.ready_var),
            ("switching", self.switching_var),
            ("timed_out", self.timeout_var),
            ("degraded", self.degraded_var),
        ]
        for index, (label, value_var) in enumerate(summary_rows):
            row = index // 3
            column = (index % 3) * 2
            ttk.Label(summary, text=label).grid(row=row, column=column, sticky="w")
            ttk.Label(summary, textvariable=value_var).grid(
                row=row, column=column + 1, padx=(4, 12), sticky="w"
            )

        ttk.Label(summary, text="detail").grid(row=5, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.detail_var).grid(
            row=5, column=1, columnspan=5, sticky="w"
        )

        backend = ttk.LabelFrame(main, text="Backend Ops", padding=8)
        backend.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            backend.columnconfigure(column, weight=1)

        ttk.Label(backend, text="joint_state_ctrl").grid(row=0, column=0, sticky="w")
        ttk.Label(backend, textvariable=self.joint_state_controller_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(backend, text="csp_ctrl").grid(row=0, column=2, sticky="w")
        ttk.Label(backend, textvariable=self.csp_controller_state_var).grid(
            row=0, column=3, sticky="w"
        )
        ttk.Label(backend, text="csv_ctrl").grid(row=0, column=4, sticky="w")
        ttk.Label(backend, textvariable=self.csv_controller_state_var).grid(
            row=0, column=5, sticky="w"
        )
        ttk.Button(
            backend,
            text="Refresh Controllers",
            command=self.refresh_controller_states,
        ).grid(row=0, column=6, padx=(8, 0), sticky="w")

        for index, service_name in enumerate(LIFECYCLE_SERVICES):
            ttk.Button(
                backend,
                text=service_name,
                command=lambda name=service_name: self.call_lifecycle_service(name),
            ).grid(row=1, column=index, padx=2, pady=(8, 0), sticky="w")

        ttk.Label(backend, text="backend_detail").grid(
            row=2, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Label(backend, textvariable=self.backend_detail_var).grid(
            row=2, column=1, columnspan=7, sticky="w", pady=(8, 0)
        )

        control = ttk.LabelFrame(main, text="Mode Control", padding=8)
        control.grid(row=2, column=0, sticky="ew", pady=(10, 0))
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

        ttk.Label(control, text="traj_time").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
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
        joints.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        for column in range(11):
            joints.columnconfigure(column, weight=1)

        headers = [
            "joint",
            "online",
            "enabled",
            "fault",
            "hb_lost",
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
            self.heartbeat_vars[joint_name] = tk.StringVar(value="-")
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
            ttk.Label(joints, textvariable=self.heartbeat_vars[joint_name]).grid(
                row=row, column=4, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.measured_vars[joint_name]).grid(
                row=row, column=5, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.reference_vars[joint_name]).grid(
                row=row, column=6, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.commanded_vel_vars[joint_name]).grid(
                row=row, column=7, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.runtime_lifecycle_vars[joint_name]).grid(
                row=row, column=8, sticky="w", padx=4
            )
            ttk.Entry(
                joints,
                textvariable=self.position_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=9, sticky="ew", padx=4)
            ttk.Entry(
                joints,
                textvariable=self.velocity_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=10, sticky="ew", padx=4)

        actions = ttk.Frame(main, padding=(0, 10, 0, 0))
        actions.grid(row=4, column=0, sticky="ew")
        ttk.Button(
            actions,
            text="Send Position Trajectory",
            command=self.on_send_trajectory,
        ).grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Button(
            actions,
            text="Send Velocity Jog",
            command=self.on_send_jog,
        ).grid(row=0, column=1, padx=(0, 8), sticky="w")
        ttk.Label(actions, textvariable=self.status_var).grid(row=0, column=2, sticky="w")

    def run(self) -> None:
        self.root.mainloop()

    def on_close(self) -> None:
        self.closed = True
        try:
            if self.state_sub is not None:
                self.state_sub.unregister()
        except Exception:
            pass
        try:
            if self.runtime_sub is not None:
                self.runtime_sub.unregister()
        except Exception:
            pass
        try:
            if self.diagnostics_sub is not None:
                self.diagnostics_sub.unregister()
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

    def apply_lifecycle_observation(self, state: Optional[str], source: str) -> None:
        if not state or state == "Unknown":
            return
        with self.state_lock:
            self.lifecycle_estimate = state
            self.lifecycle_source = source

    def infer_lifecycle_state_from_service_result(
        self, service_name: str, success: bool, message: str
    ) -> Optional[str]:
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
        return self.backend_service_ns + "/" + service_name

    def call_lifecycle_service(self, service_name: str) -> None:
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

    def on_switch_profile(self) -> None:
        threading.Thread(
            target=self._call_profile_service_worker,
            args=(self.profile_value_var.get(),),
            daemon=True,
        ).start()

    def _call_profile_service_worker(self, profile: str) -> None:
        try:
            rospy.wait_for_service(self.profile_service, timeout=2.0)
            proxy = rospy.ServiceProxy(self.profile_service, SetControlProfile)
            response = proxy(profile=profile)
            prefix = "OK" if response.success else "FAIL"
            self.set_status(f"profile={profile}: {prefix} {response.message}")
        except Exception as exc:
            self.set_status(f"profile={profile}: ERROR {exc}")

    def on_switch_linkage(self) -> None:
        threading.Thread(
            target=self._call_linkage_service_worker,
            args=(self.linkage_value_var.get(),),
            daemon=True,
        ).start()

    def _call_linkage_service_worker(self, linkage: str) -> None:
        try:
            rospy.wait_for_service(self.linkage_service, timeout=2.0)
            proxy = rospy.ServiceProxy(self.linkage_service, SetLinkageMode)
            response = proxy(mode=linkage)
            prefix = "OK" if response.success else "FAIL"
            self.set_status(f"linkage={linkage}: {prefix} {response.message}")
        except Exception as exc:
            self.set_status(f"linkage={linkage}: ERROR {exc}")

    def refresh_controller_states(self) -> None:
        self._maybe_poll_controller_states(force=True)

    def _maybe_poll_controller_states(self, force: bool = False) -> None:
        now = time.monotonic()
        with self.state_lock:
            if self.controller_poll_inflight:
                return
            if (
                not force
                and (now - self.last_controller_poll_monotonic)
                < self.controller_poll_interval_sec
            ):
                return
            self.controller_poll_inflight = True
            self.last_controller_poll_monotonic = now

        threading.Thread(
            target=self._poll_controller_states_worker,
            daemon=True,
        ).start()

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
        lifecycle = canonical_lifecycle_name(msg.lifecycle_state)
        if lifecycle and lifecycle != "Unknown":
            self.apply_lifecycle_observation(lifecycle, "manager")

    def on_runtime_state(self, msg: JointRuntimeStateArray) -> None:
        updated = {}
        first_lifecycle = ""
        for state in msg.states:
            updated[state.joint_name] = state
            if not first_lifecycle and state.lifecycle_state:
                first_lifecycle = canonical_lifecycle_name(state.lifecycle_state)
        with self.state_lock:
            self.runtime_states = updated
        if first_lifecycle and first_lifecycle != "Unknown":
            self.apply_lifecycle_observation(first_lifecycle, "runtime")

    def match_joint_name(self, status_name: str) -> str:
        for joint_name in self.joint_names:
            if status_name == joint_name:
                return joint_name
            if status_name.endswith(joint_name):
                return joint_name
        return ""

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        updated: Dict[str, CanopenDiagnosticState] = {}
        for status in msg.status:
            joint_name = self.match_joint_name(status.name)
            if not joint_name:
                continue

            kv = {item.key: item.value for item in status.values}
            updated[joint_name] = CanopenDiagnosticState(
                summary=f"{level_text(status.level)}:{status.message}",
                operational=parse_boolish(kv.get("is_operational", "false")),
                fault=parse_boolish(kv.get("is_fault", "false")),
                heartbeat_lost=parse_boolish(kv.get("heartbeat_lost_flag", "false")),
            )

        if not updated:
            return

        with self.state_lock:
            merged = dict(self.canopen_diagnostics)
            merged.update(updated)
            self.canopen_diagnostics = merged

        if any(item.fault or item.heartbeat_lost for item in updated.values()):
            self.apply_lifecycle_observation("Faulted", "diagnostics")

    @staticmethod
    def format_number(value: float) -> str:
        return f"{value:.4f}"

    def summarize_runtime_states(self, runtime_states: Dict[str, object]) -> str:
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

    def summarize_canopen_diagnostics(
        self, diagnostics: Dict[str, CanopenDiagnosticState]
    ) -> str:
        if not diagnostics:
            return "waiting for diagnostics"

        fault_joints = sorted(
            joint_name for joint_name, state in diagnostics.items() if state.fault
        )
        heartbeat_joints = sorted(
            joint_name for joint_name, state in diagnostics.items() if state.heartbeat_lost
        )
        not_operational_joints = sorted(
            joint_name
            for joint_name, state in diagnostics.items()
            if not state.operational and not state.fault and not state.heartbeat_lost
        )
        if fault_joints:
            return "fault: " + ", ".join(fault_joints)
        if heartbeat_joints:
            return "heartbeat_lost: " + ", ".join(heartbeat_joints)
        if not_operational_joints:
            return "not operational: " + ", ".join(not_operational_joints)
        return "all operational"

    def resolve_hybrid_lifecycle(
        self,
        flipper_state: Optional[FlipperControlState],
        runtime_states: Dict[str, object],
        estimate: str,
        estimate_source: str,
    ) -> tuple[str, str, str]:
        lifecycles = sorted(
            {
                canonical_lifecycle_name(getattr(state, "lifecycle_state", ""))
                for state in runtime_states.values()
                if getattr(state, "lifecycle_state", "")
            }
        )
        lifecycles = [state for state in lifecycles if state != "Unknown"]
        if len(lifecycles) == 1:
            return lifecycles[0], "runtime", self.summarize_runtime_states(runtime_states)
        if len(lifecycles) > 1:
            return "Mixed", "runtime", self.summarize_runtime_states(runtime_states)

        if flipper_state is not None:
            lifecycle = canonical_lifecycle_name(flipper_state.lifecycle_state)
            if lifecycle and lifecycle != "Unknown":
                return lifecycle, "manager", self.summarize_runtime_states(runtime_states)

        if estimate:
            return estimate, estimate_source, self.summarize_runtime_states(runtime_states)

        return "-", "-", self.summarize_runtime_states(runtime_states)

    def resolve_canopen_lifecycle(
        self,
        flipper_state: Optional[FlipperControlState],
        diagnostics: Dict[str, CanopenDiagnosticState],
        estimate: str,
        estimate_source: str,
    ) -> tuple[str, str, str]:
        detail = self.summarize_canopen_diagnostics(diagnostics)
        if any(item.fault or item.heartbeat_lost for item in diagnostics.values()):
            return "Faulted", "diagnostics", detail

        if flipper_state is not None:
            lifecycle = canonical_lifecycle_name(flipper_state.lifecycle_state)
            if lifecycle and lifecycle != "Unknown":
                return lifecycle, "manager", detail

        if estimate:
            return estimate, estimate_source, detail

        if diagnostics:
            if self.canopen_auto_release:
                return "Running", "estimated:auto_startup", detail
            if self.canopen_auto_enable or self.canopen_auto_init:
                return "Armed", "estimated:auto_startup", detail
            return "Unknown", "estimated:diagnostics", detail

        if not self.canopen_auto_init:
            return "Configured", "estimated:auto_startup", detail
        if self.canopen_auto_release:
            return "Running", "estimated:auto_startup", detail
        return "Armed", "estimated:auto_startup", detail

    def refresh_ui(self) -> None:
        self._maybe_poll_controller_states()

        with self.state_lock:
            flipper_state = self.flipper_state
            runtime_states = dict(self.runtime_states)
            canopen_diagnostics = dict(self.canopen_diagnostics)
            controller_states = dict(self.controller_states)
            status_text = self.status_text
            estimate = self.lifecycle_estimate
            estimate_source = self.lifecycle_source

        if self.backend_type == "hybrid":
            lifecycle, lifecycle_source, backend_detail = self.resolve_hybrid_lifecycle(
                flipper_state, runtime_states, estimate, estimate_source
            )
        else:
            lifecycle, lifecycle_source, backend_detail = self.resolve_canopen_lifecycle(
                flipper_state, canopen_diagnostics, estimate, estimate_source
            )

        self.status_var.set(status_text)
        self.backend_type_var.set(self.backend_type)
        self.backend_ns_var.set(self.backend_service_ns)
        self.lifecycle_var.set(lifecycle)
        self.lifecycle_source_var.set(lifecycle_source)
        self.backend_detail_var.set(backend_detail)
        self.joint_state_controller_var.set(
            controller_states.get(self.controllers["joint_state_controller"], "-")
        )
        self.csp_controller_state_var.set(
            controller_states.get(self.controllers["csp"], "-")
        )
        self.csv_controller_state_var.set(
            controller_states.get(self.controllers["csv"], "-")
        )

        if flipper_state is not None:
            self.active_profile_var.set(flipper_state.active_profile or "-")
            self.active_hardware_mode_var.set(flipper_state.active_hardware_mode or "-")
            self.active_controller_var.set(flipper_state.active_controller or "-")
            self.active_linkage_var.set(flipper_state.linkage_mode or "-")
            self.switch_state_var.set(flipper_state.switch_state or "-")
            self.ready_var.set(str(flipper_state.ready))
            self.switching_var.set(str(flipper_state.switching))
            self.timeout_var.set(str(flipper_state.command_timed_out))
            self.degraded_var.set(str(flipper_state.degraded))
            self.detail_var.set(flipper_state.detail or "-")

            measured = dict(
                zip(flipper_state.joint_names, flipper_state.measured_positions)
            )
            reference = dict(
                zip(flipper_state.joint_names, flipper_state.reference_positions)
            )
            commanded_vel = dict(
                zip(flipper_state.joint_names, flipper_state.commanded_velocities)
            )
        else:
            self.active_profile_var.set("-")
            self.active_hardware_mode_var.set("-")
            self.active_controller_var.set("-")
            self.active_linkage_var.set("-")
            self.switch_state_var.set("-")
            self.ready_var.set("-")
            self.switching_var.set("-")
            self.timeout_var.set("-")
            self.degraded_var.set("-")
            self.detail_var.set("-")
            measured = {}
            reference = {}
            commanded_vel = {}

        for joint_name in self.joint_names:
            if joint_name in measured:
                self.measured_vars[joint_name].set(
                    self.format_number(measured[joint_name])
                )
            else:
                self.measured_vars[joint_name].set("-")

            if joint_name in reference:
                self.reference_vars[joint_name].set(
                    self.format_number(reference[joint_name])
                )
            else:
                self.reference_vars[joint_name].set("-")

            if joint_name in commanded_vel:
                self.commanded_vel_vars[joint_name].set(
                    self.format_number(commanded_vel[joint_name])
                )
            else:
                self.commanded_vel_vars[joint_name].set("-")

            if self.backend_type == "hybrid":
                runtime = runtime_states.get(joint_name)
                if runtime is None:
                    self.online_vars[joint_name].set("-")
                    self.enabled_vars[joint_name].set("-")
                    self.fault_vars[joint_name].set("-")
                    self.heartbeat_vars[joint_name].set("-")
                    self.runtime_lifecycle_vars[joint_name].set("-")
                else:
                    self.online_vars[joint_name].set("1" if runtime.online else "0")
                    self.enabled_vars[joint_name].set("1" if runtime.enabled else "0")
                    self.fault_vars[joint_name].set("1" if runtime.fault else "0")
                    self.heartbeat_vars[joint_name].set("-")
                    self.runtime_lifecycle_vars[joint_name].set(
                        canonical_lifecycle_name(runtime.lifecycle_state)
                    )
                continue

            diagnostic = canopen_diagnostics.get(joint_name)
            if diagnostic is None:
                self.online_vars[joint_name].set("-")
                self.enabled_vars[joint_name].set("-")
                self.fault_vars[joint_name].set("-")
                self.heartbeat_vars[joint_name].set("-")
                self.runtime_lifecycle_vars[joint_name].set(lifecycle)
                continue

            enabled = (
                lifecycle in {"Armed", "Running"}
                and diagnostic.operational
                and not diagnostic.fault
                and not diagnostic.heartbeat_lost
            )
            self.online_vars[joint_name].set("1" if diagnostic.operational else "0")
            self.enabled_vars[joint_name].set("1" if enabled else "0")
            self.fault_vars[joint_name].set("1" if diagnostic.fault else "0")
            self.heartbeat_vars[joint_name].set(
                "1" if diagnostic.heartbeat_lost else "0"
            )
            self.runtime_lifecycle_vars[joint_name].set(lifecycle)

        if not self.closed and not rospy.is_shutdown():
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
        help="Namespace for hybrid backend services and runtime topics.",
    )
    parser.add_argument(
        "--canopen-ns",
        default=DEFAULT_CANOPEN_NS,
        help="Namespace for CANopen backend services.",
    )
    parser.add_argument(
        "--backend-type",
        default="auto",
        choices=["auto", "hybrid", "canopen"],
        help="Override backend type. Default: read from flipper_control/backend_type.",
    )
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()
    rospy.init_node("flipper_motor_debug_ui", anonymous=True, disable_signals=True)
    ui = FlipperMotorDebugUi(
        args.flipper_ns,
        args.hybrid_ns,
        args.canopen_ns,
        args.backend_type,
    )
    ui.run()


if __name__ == "__main__":
    main()
