#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from can_driver.msg import MotorState
from can_driver.srv import MotorCommand, MotorCommandRequest


class CanDriverUiBridge:
    def __init__(self):
        self.backend_ns = rospy.get_param("~backend_ns", "")
        self.backend_ns = self.backend_ns.rstrip("/")

        self.backend_motor_command = self._resolve("motor_command")
        self.backend_motor_states = self._resolve("motor_states")

        self.motor_cfg = rospy.get_param("~motors", [])
        self.name_to_cfg = {}

        for item in self.motor_cfg:
            name = item.get("name", "")
            if not name:
                continue
            self.name_to_cfg[name] = item

        # Proxy service for UI: stable endpoint regardless of backend namespace changes
        self.proxy_srv = rospy.Service("~motor_command", MotorCommand, self._on_motor_command)
        self.backend_srv = rospy.ServiceProxy(self.backend_motor_command, MotorCommand)

        # Relay backend motor states to private namespace for UI consumption
        self.state_pub = rospy.Publisher("~motor_states", MotorState, queue_size=50)
        self.state_sub = rospy.Subscriber(self.backend_motor_states, MotorState, self._on_motor_state, queue_size=50)

        # Optional per-joint direct command ingress for UI
        self.cmd_pubs_vel = {}
        self.cmd_pubs_pos = {}
        self.cmd_subs = []
        for name, cfg in self.name_to_cfg.items():
            mode = str(cfg.get("mode", "")).strip().lower()
            if mode == "velocity":
                out_topic = self._resolve("motor/{}/cmd_velocity".format(name))
                in_topic = "~motor/{}/cmd_velocity".format(name)
                self.cmd_pubs_vel[name] = rospy.Publisher(out_topic, Float64, queue_size=10)
                self.cmd_subs.append(rospy.Subscriber(in_topic, Float64, self._mk_vel_cb(name), queue_size=10))
            elif mode == "position":
                out_topic = self._resolve("motor/{}/cmd_position".format(name))
                in_topic = "~motor/{}/cmd_position".format(name)
                self.cmd_pubs_pos[name] = rospy.Publisher(out_topic, Float64, queue_size=10)
                self.cmd_subs.append(rospy.Subscriber(in_topic, Float64, self._mk_pos_cb(name), queue_size=10))

        rospy.loginfo("[can_driver_ui] bridge ready. backend_ns='%s' service='%s' states='%s'",
                      self.backend_ns, self.backend_motor_command, self.backend_motor_states)

    def _resolve(self, name):
        if self.backend_ns:
            return "{}/{}".format(self.backend_ns, name)
        return "/{}".format(name)

    def _clamp(self, name, value):
        cfg = self.name_to_cfg.get(name, {})
        min_v = cfg.get("min", None)
        max_v = cfg.get("max", None)
        if min_v is not None and value < min_v:
            return float(min_v)
        if max_v is not None and value > max_v:
            return float(max_v)
        return value

    def _mk_vel_cb(self, name):
        def cb(msg):
            out = Float64()
            out.data = self._clamp(name, float(msg.data))
            self.cmd_pubs_vel[name].publish(out)
        return cb

    def _mk_pos_cb(self, name):
        def cb(msg):
            out = Float64()
            out.data = self._clamp(name, float(msg.data))
            self.cmd_pubs_pos[name].publish(out)
        return cb

    def _on_motor_state(self, msg):
        self.state_pub.publish(msg)

    def _on_motor_command(self, req):
        try:
            self.backend_srv.wait_for_service(timeout=1.0)
            return self.backend_srv(req)
        except rospy.ROSException:
            return self._fail("backend motor_command service timeout")
        except rospy.ServiceException as exc:
            return self._fail("backend motor_command call failed: {}".format(exc))

    @staticmethod
    def _fail(message):
        res = MotorCommand._response_class()
        res.success = False
        res.message = message
        return res


if __name__ == "__main__":
    rospy.init_node("can_driver_ui_bridge")
    CanDriverUiBridge()
    rospy.spin()
