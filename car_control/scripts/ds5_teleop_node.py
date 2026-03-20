#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


class DS5TeleopNode(object):
    def __init__(self):
        # Topics
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.chassis_topic = rospy.get_param('~chassis_cmd_topic', '/car_control/cmd_vel')
        self.servo_topic = rospy.get_param('~servo_cmd_topic', '/servo_server/delta_twist_cmds')
        self.gripper_open_topic = rospy.get_param('~gripper_open_topic', '/car_control/gripper_open')

        # Servo frame
        self.servo_frame = rospy.get_param('~servo_frame', 'base_link_root')

        # Scales
        self.max_chassis_vx = float(rospy.get_param('~max_chassis_vx', 0.8))
        self.max_chassis_wz = float(rospy.get_param('~max_chassis_wz', 1.5))
        self.max_arm_linear = float(rospy.get_param('~max_arm_linear', 0.15))
        self.max_arm_angular = float(rospy.get_param('~max_arm_angular', 0.6))

        # Safety
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.25))
        self.deadzone = float(rospy.get_param('~deadzone', 0.12))

        # Axis/button indices (configurable for DS4/DS5 differences)
        self.AXIS_L_X = int(rospy.get_param('~axis_lx', 0))
        self.AXIS_L_Y = int(rospy.get_param('~axis_ly', 1))
        self.AXIS_L2 = int(rospy.get_param('~axis_l2', 2))
        self.AXIS_R_X = int(rospy.get_param('~axis_rx', 3))
        self.AXIS_R_Y = int(rospy.get_param('~axis_ry', 4))
        self.AXIS_R2 = int(rospy.get_param('~axis_r2', 5))
        self.AXIS_DPAD_X = int(rospy.get_param('~axis_dpad_x', 6))
        self.AXIS_DPAD_Y = int(rospy.get_param('~axis_dpad_y', 7))

        self.BTN_SQUARE = int(rospy.get_param('~btn_square', 0))
        self.BTN_CROSS = int(rospy.get_param('~btn_cross', 1))
        self.BTN_CIRCLE = int(rospy.get_param('~btn_circle', 2))
        self.BTN_TRIANGLE = int(rospy.get_param('~btn_triangle', 3))
        self.BTN_L1 = int(rospy.get_param('~btn_l1', 4))
        self.BTN_R1 = int(rospy.get_param('~btn_r1', 5))
        self.BTN_OPTIONS = int(rospy.get_param('~btn_options', 9))

        # Mode
        self.MODE_CHASSIS = 0
        self.MODE_ARM = 1
        self.mode = self.MODE_CHASSIS
        self.last_options = 0
        self.last_square = 0
        self.last_circle = 0

        self.last_joy_time = rospy.Time(0)

        # Pub/Sub
        self.pub_chassis = rospy.Publisher(self.chassis_topic, Twist, queue_size=10)
        self.pub_servo = rospy.Publisher(self.servo_topic, TwistStamped, queue_size=10)
        self.pub_gripper_open = rospy.Publisher(self.gripper_open_topic, Bool, queue_size=10)
        self.sub = rospy.Subscriber(self.joy_topic, Joy, self.joy_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_cb)

        rospy.loginfo('ds5_teleop_node started, mode=CHASSIS')

    def axis(self, msg, idx):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        v = msg.axes[idx]
        return 0.0 if abs(v) < self.deadzone else v

    def button(self, msg, idx):
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return msg.buttons[idx]

    def trigger_to_01(self, axis_val):
        # common joy mapping: released=1, pressed=-1
        return (1.0 - axis_val) * 0.5

    def publish_zero(self):
        self.pub_chassis.publish(Twist())
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self.servo_frame
        self.pub_servo.publish(ts)

    def joy_cb(self, msg):
        now = rospy.Time.now()
        self.last_joy_time = now

        # Mode switch on OPTIONS rising edge
        options = self.button(msg, self.BTN_OPTIONS)
        if options == 1 and self.last_options == 0:
            self.mode = self.MODE_ARM if self.mode == self.MODE_CHASSIS else self.MODE_CHASSIS
            rospy.loginfo('ds5_teleop_node mode=%s', 'ARM_SERVO' if self.mode == self.MODE_ARM else 'CHASSIS')
            self.publish_zero()
        self.last_options = options

        # Gripper edge control: square=open, circle=close
        square = self.button(msg, self.BTN_SQUARE)
        circle = self.button(msg, self.BTN_CIRCLE)
        if square == 1 and self.last_square == 0:
            self.pub_gripper_open.publish(Bool(data=True))
        if circle == 1 and self.last_circle == 0:
            self.pub_gripper_open.publish(Bool(data=False))
        self.last_square = square
        self.last_circle = circle

        if self.mode == self.MODE_CHASSIS:
            cmd = Twist()
            cmd.linear.x = self.axis(msg, self.AXIS_L_Y) * self.max_chassis_vx
            cmd.angular.z = self.axis(msg, self.AXIS_L_X) * self.max_chassis_wz
            self.pub_chassis.publish(cmd)

            # keep servo side zeroed
            ts = TwistStamped()
            ts.header.stamp = now
            ts.header.frame_id = self.servo_frame
            self.pub_servo.publish(ts)
            return

        # ARM_SERVO mode
        ts = TwistStamped()
        ts.header.stamp = now
        ts.header.frame_id = self.servo_frame

        # Linear: LX->y, LY->z, triggers+DPAD->x
        ts.twist.linear.y = self.axis(msg, self.AXIS_L_X) * self.max_arm_linear
        ts.twist.linear.z = self.axis(msg, self.AXIS_L_Y) * self.max_arm_linear

        l2 = self.trigger_to_01(self.axis(msg, self.AXIS_L2))
        r2 = self.trigger_to_01(self.axis(msg, self.AXIS_R2))
        dpad_y = self.axis(msg, self.AXIS_DPAD_Y)
        # forward/back on x: R2 forward, L2 backward; D-Pad down inverts sign quickly
        x_cmd = (r2 - l2) * self.max_arm_linear
        if dpad_y < -0.5:
            x_cmd = -x_cmd
        ts.twist.linear.x = x_cmd

        # Angular: RX->yaw(z), RY->pitch(y), L1/R1->roll(x)
        ts.twist.angular.y = self.axis(msg, self.AXIS_R_Y) * self.max_arm_angular
        ts.twist.angular.z = self.axis(msg, self.AXIS_R_X) * self.max_arm_angular
        if self.button(msg, self.BTN_L1):
            ts.twist.angular.x = self.max_arm_angular
        elif self.button(msg, self.BTN_R1):
            ts.twist.angular.x = -self.max_arm_angular

        self.pub_servo.publish(ts)
        self.pub_chassis.publish(Twist())

    def watchdog_cb(self, _event):
        if self.last_joy_time == rospy.Time(0):
            return
        if (rospy.Time.now() - self.last_joy_time).to_sec() > self.cmd_timeout:
            self.publish_zero()


if __name__ == '__main__':
    rospy.init_node('ds5_teleop_node')
    DS5TeleopNode()
    rospy.spin()
