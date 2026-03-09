# can_driver_ui

`can_driver_ui` provides UI-facing bridges and tools for `can_driver`.

## 1) Bridge node
- service proxy: `~motor_command` -> backend `motor_command`
- topic relay: backend `motor_states` -> `~motor_states`
- optional ingress topics per joint:
  - `~motor/<name>/cmd_velocity`
  - `~motor/<name>/cmd_position`

Run:
```bash
roslaunch can_driver_ui ui_bridge.launch
```

## 2) Keyboard UI node
Terminal control for quick manual tests.

Keys:
- `w`: positive command
- `s`: negative command
- `space`: stop (publish 0)
- `1..9`: select motor by index
- `n` / `p`: next / previous motor
- `e`: enable selected motor
- `d`: disable selected motor
- `x`: stop selected motor (service)
- `q`: quit

Run:
```bash
roslaunch can_driver_ui keyboard_ui.launch
```

## 3) GUI UI node (Tkinter)
Provides a real GUI and allows explicit endpoint selection.

Capabilities:
- manual input of service topic and command topic
- motor selection from config
- auto-generate topic template from `bridge_ns + motor + mode`
- keyboard control inside GUI window: `w/s/space`
- button control: forward/reverse/stop + enable/disable/stop service

Run:
```bash
roslaunch can_driver_ui gui_ui.launch
```

## 4) Endpoints for frontend
Assuming bridge node is `can_driver_ui_bridge`:
- service: `/can_driver_ui_bridge/motor_command`
- state topic: `/can_driver_ui_bridge/motor_states`
- command topics:
  - `/can_driver_ui_bridge/motor/<name>/cmd_velocity`
  - `/can_driver_ui_bridge/motor/<name>/cmd_position`

## 5) Which package decides controllers
- `can_driver/config/can_driver.yaml`: joint interface type (`velocity` or `position`).
- `can_driver/config/ros_controllers.yaml`: controller definitions.
- `can_driver/launch/can_driver.launch` spawner args: controllers actually started.

Current default in this workspace:
- only `joint_state_controller` is started by default.
- `arm_controller` / `wheel_controller` are present but not started unless you enable them in launch.

## 6) Parity status vs ros_canopen (protocol ignored)
Current status is "close in architecture, not fully identical in user experience".
- aligned: `hardware_interface + controller_manager` stack and direct motor command path.
- gaps: default controller startup completeness, MoveIt mapping completeness, and reliability hardening (state/recovery/arbitration).
