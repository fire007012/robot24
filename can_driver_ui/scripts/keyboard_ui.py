#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import Float64
from can_driver.srv import MotorCommand, MotorCommandRequest


HELP = """
can_driver keyboard UI

Keys:
  w : forward / positive command
  s : reverse / negative command
  <space> : stop (publish 0)
  1..9 : select motor by index
  n / p : next / previous motor
  e : enable selected motor (service)
  d : disable selected motor (service)
  x : stop selected motor (service CMD_STOP)
  q : quit
""".strip()


class KeyboardUi:
    def __init__(self):
        self.bridge_ns = rospy.get_param("~bridge_ns", "/can_driver_ui_bridge").rstrip("/")
        self.motors = rospy.get_param("~motors", [])
        self.velocity_value = float(rospy.get_param("~velocity_value", 30.0))

        if not self.motors:
            raise RuntimeError("~motors is empty. load can_driver_ui/config/motors.yaml")

        self.selected = 0
        self.pub_map = {}

        for item in self.motors:
            name = item.get("name", "")
            mode = str(item.get("mode", "velocity")).strip().lower()
            if not name:
                continue
            if mode == "position":
                topic = "{}/motor/{}/cmd_position".format(self.bridge_ns, name)
            else:
                topic = "{}/motor/{}/cmd_velocity".format(self.bridge_ns, name)
            self.pub_map[name] = rospy.Publisher(topic, Float64, queue_size=10)

        self.srv = rospy.ServiceProxy("{}/motor_command".format(self.bridge_ns), MotorCommand)

        rospy.loginfo("[keyboard_ui] started, bridge_ns=%s, motors=%d", self.bridge_ns, len(self.motors))
        rospy.loginfo("\n%s", HELP)
        self.print_status()

    def print_status(self):
        m = self.motors[self.selected]
        rospy.loginfo("[keyboard_ui] selected #%d name=%s id=%s mode=%s",
                      self.selected + 1,
                      m.get("name", "?"),
                      m.get("motor_id", "?"),
                      m.get("mode", "velocity"))

    def select_index(self, idx):
        if idx < 0 or idx >= len(self.motors):
            return
        self.selected = idx
        self.print_status()

    def selected_motor(self):
        return self.motors[self.selected]

    def publish_value(self, value):
        m = self.selected_motor()
        name = m.get("name", "")
        min_v = m.get("min", None)
        max_v = m.get("max", None)
        if min_v is not None and value < min_v:
            value = float(min_v)
        if max_v is not None and value > max_v:
            value = float(max_v)

        pub = self.pub_map.get(name)
        if not pub:
            rospy.logwarn("[keyboard_ui] no publisher for motor '%s'", name)
            return

        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)
        rospy.loginfo("[keyboard_ui] publish %s -> %.3f", name, msg.data)

    def call_motor_cmd(self, command):
        m = self.selected_motor()
        req = MotorCommandRequest()
        req.motor_id = int(m.get("motor_id", 0))
        req.command = command
        req.value = 0.0
        try:
            self.srv.wait_for_service(timeout=1.0)
            res = self.srv(req)
            rospy.loginfo("[keyboard_ui] motor_command id=%d cmd=%d success=%s msg=%s",
                          req.motor_id, req.command, str(res.success), res.message)
        except rospy.ROSException:
            rospy.logerr("[keyboard_ui] motor_command service timeout")
        except rospy.ServiceException as exc:
            rospy.logerr("[keyboard_ui] motor_command call failed: %s", str(exc))

    def handle_key(self, c):
        if c == "w":
            self.publish_value(+self.velocity_value)
        elif c == "s":
            self.publish_value(-self.velocity_value)
        elif c == " ":
            self.publish_value(0.0)
        elif c == "n":
            self.select_index((self.selected + 1) % len(self.motors))
        elif c == "p":
            self.select_index((self.selected - 1 + len(self.motors)) % len(self.motors))
        elif c == "e":
            self.call_motor_cmd(MotorCommandRequest.CMD_ENABLE)
        elif c == "d":
            self.call_motor_cmd(MotorCommandRequest.CMD_DISABLE)
        elif c == "x":
            self.call_motor_cmd(MotorCommandRequest.CMD_STOP)
        elif c.isdigit():
            idx = int(c) - 1
            self.select_index(idx)


def run_keyboard_loop(ui):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if ch == "q":
                rospy.loginfo("[keyboard_ui] quit")
                break
            ui.handle_key(ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    rospy.init_node("can_driver_keyboard_ui")
    ui = KeyboardUi()
    run_keyboard_loop(ui)
