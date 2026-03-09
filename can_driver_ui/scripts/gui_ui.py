#!/usr/bin/env python3
import threading
import tkinter as tk
from tkinter import ttk

import rospy
from std_msgs.msg import Float64
from can_driver.msg import MotorState
from can_driver.srv import MotorCommand, MotorCommandRequest


class CanDriverGuiUi:
    def __init__(self):
        self.motors = rospy.get_param("~motors", [])
        self.default_bridge_ns = rospy.get_param("~bridge_ns", "/can_driver_ui_bridge").rstrip("/")

        self._pub = None
        self._pub_topic = ""
        self._srv = None
        self._srv_name = ""
        self._state_sub = None
        self._state_topic = ""
        self._lock = threading.Lock()
        self._latest_states_by_name = {}
        self._latest_states_by_id = {}
        self._suppress_slider_event = False

        self.root = tk.Tk()
        self.root.title("CAN 电机控制界面")
        self.root.geometry("900x620")

        self._build_ui()
        self._bind_keys()
        self._refresh_motor_list()
        self._apply_template_topics()
        self._update_slider_for_selected_motor(reset_value=True)
        self._schedule_state_refresh()

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)

        # Endpoint section
        ep = ttk.LabelFrame(outer, text="接口配置", padding=10)
        ep.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(ep, text="服务话题 (motor_command)").grid(row=0, column=0, sticky="w")
        self.srv_var = tk.StringVar(value=self.default_bridge_ns + "/motor_command")
        ttk.Entry(ep, textvariable=self.srv_var, width=60).grid(row=0, column=1, sticky="we", padx=8)

        ttk.Label(ep, text="命令话题 (Float64)").grid(row=1, column=0, sticky="w")
        self.cmd_var = tk.StringVar(value=self.default_bridge_ns + "/motor/left_wheel/cmd_velocity")
        ttk.Entry(ep, textvariable=self.cmd_var, width=60).grid(row=1, column=1, sticky="we", padx=8)

        ttk.Button(ep, text="连接", command=self._connect_endpoints).grid(row=0, column=2, rowspan=2, padx=6)
        ep.columnconfigure(1, weight=1)

        # Motor selection section
        sel = ttk.LabelFrame(outer, text="电机选择", padding=10)
        sel.pack(fill=tk.X, pady=(0, 10))

        self.motor_names = [m.get("name", "") for m in self.motors if m.get("name")]
        self.motor_var = tk.StringVar(value=self.motor_names[0] if self.motor_names else "")
        ttk.Label(sel, text="电机").grid(row=0, column=0, sticky="w")
        self.motor_combo = ttk.Combobox(sel, textvariable=self.motor_var, values=self.motor_names, width=24, state="readonly")
        self.motor_combo.grid(row=0, column=1, sticky="w", padx=8)
        self.motor_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_motor_selected())

        self.mode_var = tk.StringVar(value="velocity")
        ttk.Label(sel, text="模式").grid(row=0, column=2, sticky="w")
        ttk.Radiobutton(sel, text="速度", variable=self.mode_var, value="velocity", command=self._on_mode_changed).grid(row=0, column=3, sticky="w")
        ttk.Radiobutton(sel, text="位置", variable=self.mode_var, value="position", command=self._on_mode_changed).grid(row=0, column=4, sticky="w")

        ttk.Label(sel, text="桥接命名空间").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.bridge_ns_var = tk.StringVar(value=self.default_bridge_ns)
        ttk.Entry(sel, textvariable=self.bridge_ns_var, width=28).grid(row=1, column=1, sticky="w", padx=8, pady=(8, 0))

        ttk.Button(sel, text="应用当前电机", command=self._apply_template_topics).grid(row=1, column=2, columnspan=2, sticky="w", pady=(8, 0))

        # Command section
        cmd = ttk.LabelFrame(outer, text="控制命令", padding=10)
        cmd.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(cmd, text="数值").grid(row=0, column=0, sticky="w")
        self.value_var = tk.DoubleVar(value=30.0)
        ttk.Entry(cmd, textvariable=self.value_var, width=12).grid(row=0, column=1, sticky="w", padx=8)
        ttk.Button(cmd, text="发送输入值", command=self._publish_from_entry).grid(row=0, column=2, padx=4)

        self.live_send_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(cmd, text="拖动滑条实时发送", variable=self.live_send_var).grid(row=0, column=3, columnspan=2, sticky="w")

        self.slider_title_var = tk.StringVar(value="速度/位置滑条")
        ttk.Label(cmd, textvariable=self.slider_title_var).grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.slider_value_var = tk.StringVar(value="0.000")
        ttk.Label(cmd, textvariable=self.slider_value_var).grid(row=1, column=1, sticky="w", padx=8, pady=(8, 0))

        self.slider_var = tk.DoubleVar(value=0.0)
        self.value_scale = tk.Scale(
            cmd,
            orient=tk.HORIZONTAL,
            length=460,
            resolution=1.0,
            from_=-100.0,
            to=100.0,
            variable=self.slider_var,
            showvalue=False,
            command=self._on_slider_changed,
        )
        self.value_scale.grid(row=1, column=2, columnspan=3, sticky="we", pady=(8, 0))

        ttk.Button(cmd, text="正转 (W)", command=lambda: self._publish_signed(+1)).grid(row=2, column=2, padx=4, pady=(8, 0))
        ttk.Button(cmd, text="反转 (S)", command=lambda: self._publish_signed(-1)).grid(row=2, column=3, padx=4, pady=(8, 0))
        ttk.Button(cmd, text="停止 (空格)", command=self._publish_zero).grid(row=2, column=4, padx=4, pady=(8, 0))

        ttk.Button(cmd, text="使能", command=lambda: self._call_motor_cmd(MotorCommandRequest.CMD_ENABLE)).grid(row=3, column=2, padx=4, pady=(8, 0))
        ttk.Button(cmd, text="失能", command=lambda: self._call_motor_cmd(MotorCommandRequest.CMD_DISABLE)).grid(row=3, column=3, padx=4, pady=(8, 0))
        ttk.Button(cmd, text="急停指令", command=lambda: self._call_motor_cmd(MotorCommandRequest.CMD_STOP)).grid(row=3, column=4, padx=4, pady=(8, 0))

        # State section
        st = ttk.LabelFrame(outer, text="电机状态", padding=10)
        st.pack(fill=tk.X, pady=(0, 10))

        self.state_pos_var = tk.StringVar(value="--")
        self.state_vel_var = tk.StringVar(value="--")
        self.state_cur_var = tk.StringVar(value="--")
        self.state_mode_var = tk.StringVar(value="--")
        self.state_en_var = tk.StringVar(value="--")
        self.state_fault_var = tk.StringVar(value="--")
        self.state_age_var = tk.StringVar(value="--")

        ttk.Label(st, text="位置").grid(row=0, column=0, sticky="w")
        ttk.Label(st, textvariable=self.state_pos_var).grid(row=0, column=1, sticky="w", padx=(4, 16))
        ttk.Label(st, text="速度").grid(row=0, column=2, sticky="w")
        ttk.Label(st, textvariable=self.state_vel_var).grid(row=0, column=3, sticky="w", padx=(4, 16))
        ttk.Label(st, text="电流").grid(row=0, column=4, sticky="w")
        ttk.Label(st, textvariable=self.state_cur_var).grid(row=0, column=5, sticky="w", padx=(4, 16))

        ttk.Label(st, text="模式").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(st, textvariable=self.state_mode_var).grid(row=1, column=1, sticky="w", padx=(4, 16), pady=(6, 0))
        ttk.Label(st, text="使能").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Label(st, textvariable=self.state_en_var).grid(row=1, column=3, sticky="w", padx=(4, 16), pady=(6, 0))
        ttk.Label(st, text="故障").grid(row=1, column=4, sticky="w", pady=(6, 0))
        ttk.Label(st, textvariable=self.state_fault_var).grid(row=1, column=5, sticky="w", padx=(4, 16), pady=(6, 0))

        ttk.Label(st, text="状态时延").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Label(st, textvariable=self.state_age_var).grid(row=2, column=1, sticky="w", pady=(6, 0))

        # Log section
        logf = ttk.LabelFrame(outer, text="日志", padding=10)
        logf.pack(fill=tk.BOTH, expand=True)

        self.log = tk.Text(logf, height=12)
        self.log.pack(fill=tk.BOTH, expand=True)

        self._append("切换电机会自动应用并连接。\n键盘快捷键：w/s/空格")

    def _bind_keys(self):
        self.root.bind("<KeyPress-w>", lambda _e: self._publish_signed(+1))
        self.root.bind("<KeyPress-s>", lambda _e: self._publish_signed(-1))
        self.root.bind("<KeyPress-space>", lambda _e: self._publish_zero())

    def _refresh_motor_list(self):
        if not self.motor_names:
            self._append("No motors configured in ~motors")
            return

    def _on_motor_selected(self):
        m = self._selected_motor()
        if m:
            mode = str(m.get("mode", "")).strip().lower()
            if mode in ("velocity", "position"):
                self.mode_var.set(mode)
        self._apply_template_topics()
        self._update_slider_for_selected_motor(reset_value=True)
        self._connect_endpoints()

    def _on_mode_changed(self):
        self._apply_template_topics()
        self._update_slider_for_selected_motor(reset_value=True)
        self._connect_endpoints()

    def _selected_motor(self):
        name = self.motor_var.get().strip()
        for m in self.motors:
            if m.get("name") == name:
                return m
        return None

    def _apply_template_topics(self):
        m = self._selected_motor()
        if not m:
            return

        bridge_ns = self.bridge_ns_var.get().strip().rstrip("/")
        if not bridge_ns:
            bridge_ns = "/can_driver_ui_bridge"

        mode = self.mode_var.get().strip().lower()
        if mode not in ("velocity", "position"):
            mode = str(m.get("mode", "velocity")).strip().lower()

        suffix = "cmd_velocity" if mode == "velocity" else "cmd_position"
        cmd_topic = "{}/motor/{}/{}".format(bridge_ns, m.get("name"), suffix)
        srv_topic = "{}/motor_command".format(bridge_ns)

        self.cmd_var.set(cmd_topic)
        self.srv_var.set(srv_topic)
        self._append("Applied motor={} mode={} -> {}".format(m.get("name"), mode, cmd_topic))

    def _update_slider_for_selected_motor(self, reset_value):
        m = self._selected_motor()
        if not m:
            return

        mode = self.mode_var.get().strip().lower()
        if mode not in ("velocity", "position"):
            mode = str(m.get("mode", "velocity")).strip().lower()

        min_v = float(m.get("min", -100.0))
        max_v = float(m.get("max", 100.0))
        if min_v >= max_v:
            min_v, max_v = -100.0, 100.0
        step = float(m.get("step", 1.0))
        if step <= 0:
            step = 1.0

        self.slider_title_var.set("速度滑条" if mode == "velocity" else "位置滑条")
        self.value_scale.configure(from_=min_v, to=max_v, resolution=step)

        if reset_value:
            target = 0.0 if (min_v <= 0.0 <= max_v) else min_v
            self._set_command_value(target)
            self._append("Slider range [{:.3f}, {:.3f}], step {:.3f}".format(min_v, max_v, step))

    def _connect_endpoints(self):
        cmd_topic = self.cmd_var.get().strip()
        srv_name = self.srv_var.get().strip()
        bridge_ns = self.bridge_ns_var.get().strip().rstrip("/")
        if not bridge_ns:
            bridge_ns = "/can_driver_ui_bridge"
        state_topic = "{}/motor_states".format(bridge_ns)
        if not cmd_topic or not srv_name:
            self._append("ERROR: command topic / service topic is empty")
            return

        with self._lock:
            self._pub = rospy.Publisher(cmd_topic, Float64, queue_size=10)
            self._pub_topic = cmd_topic
            self._srv = rospy.ServiceProxy(srv_name, MotorCommand)
            self._srv_name = srv_name
            if self._state_sub is not None:
                self._state_sub.unregister()
            self._state_sub = rospy.Subscriber(state_topic, MotorState, self._on_motor_state, queue_size=50)
            self._state_topic = state_topic

        self._append("Connected cmd='{}' srv='{}' state='{}'".format(cmd_topic, srv_name, state_topic))

    def _publish_signed(self, sign):
        value = abs(float(self.value_var.get())) * float(sign)
        self._set_command_value(value)
        self._publish(value)

    def _publish_zero(self):
        self._set_command_value(0.0)
        self._publish(0.0)

    def _publish_from_entry(self):
        value = float(self.value_var.get())
        self._set_command_value(value)
        self._publish(value)

    def _on_slider_changed(self, value):
        if self._suppress_slider_event:
            return
        try:
            value_f = float(value)
        except ValueError:
            return
        self.value_var.set(value_f)
        self.slider_value_var.set("{:.3f}".format(value_f))
        if self.live_send_var.get():
            self._publish(value_f)

    def _set_command_value(self, value):
        value_f = float(value)
        self.value_var.set(value_f)
        self.slider_value_var.set("{:.3f}".format(value_f))
        self._suppress_slider_event = True
        self.slider_var.set(value_f)
        self._suppress_slider_event = False

    def _publish(self, value):
        with self._lock:
            pub = self._pub
            topic = self._pub_topic

        if pub is None:
            self._append("ERROR: not connected, click Connect first")
            return

        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)
        self._append("PUB {} -> {:.3f}".format(topic, msg.data))

    def _call_motor_cmd(self, command):
        m = self._selected_motor()
        if not m:
            self._append("ERROR: no selected motor")
            return

        with self._lock:
            srv = self._srv
            name = self._srv_name

        if srv is None:
            self._append("ERROR: not connected, click Connect first")
            return

        req = MotorCommandRequest()
        req.motor_id = int(m.get("motor_id", 0))
        req.command = command
        req.value = 0.0

        try:
            srv.wait_for_service(timeout=1.0)
            res = srv(req)
            self._append("SRV {} id={} cmd={} success={} msg={}".format(
                name, req.motor_id, req.command, str(res.success), res.message
            ))
        except rospy.ROSException:
            self._append("ERROR: service timeout {}".format(name))
        except rospy.ServiceException as exc:
            self._append("ERROR: service call failed {}".format(str(exc)))

    def _append(self, text):
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        rospy.loginfo("[gui_ui] %s", text)

    def _on_motor_state(self, msg):
        stamp = rospy.get_time()
        record = {
            "motor_id": int(msg.motor_id),
            "name": str(msg.name),
            "position": int(msg.position),
            "velocity": int(msg.velocity),
            "current": int(msg.current),
            "mode": int(msg.mode),
            "enabled": bool(msg.enabled),
            "fault": bool(msg.fault),
            "stamp": stamp,
        }
        with self._lock:
            name = record["name"].strip()
            if name:
                self._latest_states_by_name[name] = record
            self._latest_states_by_id[record["motor_id"]] = record

    def _schedule_state_refresh(self):
        self._update_state_view()
        self.root.after(200, self._schedule_state_refresh)

    def _update_state_view(self):
        m = self._selected_motor()
        if not m:
            self._set_state_texts("--", "--", "--", "--", "--", "--", "--")
            return

        name = str(m.get("name", "")).strip()
        motor_id = int(m.get("motor_id", 0))
        with self._lock:
            state = self._latest_states_by_name.get(name)
            if state is None:
                state = self._latest_states_by_id.get(motor_id)

        if state is None:
            self._set_state_texts("--", "--", "--", "--", "--", "--", "等待数据")
            return

        age_sec = max(0.0, rospy.get_time() - state["stamp"])
        self._set_state_texts(
            str(state["position"]),
            str(state["velocity"]),
            str(state["current"]),
            self._mode_to_text(state["mode"]),
            "ON" if state["enabled"] else "OFF",
            "YES" if state["fault"] else "NO",
            "{:.2f}s".format(age_sec),
        )

    def _mode_to_text(self, mode):
        if mode == MotorState.MODE_POSITION:
            return "POSITION"
        if mode == MotorState.MODE_VELOCITY:
            return "VELOCITY"
        return "UNKNOWN"

    def _set_state_texts(self, position, velocity, current, mode, enabled, fault, age):
        self.state_pos_var.set(position)
        self.state_vel_var.set(velocity)
        self.state_cur_var.set(current)
        self.state_mode_var.set(mode)
        self.state_en_var.set(enabled)
        self.state_fault_var.set(fault)
        self.state_age_var.set(age)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    rospy.init_node("can_driver_gui_ui", disable_signals=True)
    app = CanDriverGuiUi()
    app.run()
