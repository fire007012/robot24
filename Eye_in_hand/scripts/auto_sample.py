#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eye-in-hand 手眼标定自动采样脚本

依次把机械臂移动到 auto_poses.yaml 里的每个位姿，每到一个点：
  1. 等 aruco marker 出现在相机视野中（/aruco_single/pose 有新消息）
  2. 调用 easy_handeye 的 take_sample 服务记录样本

全部采完后会自动调用 compute + save，结果保存到
  ~/.ros/easy_handeye/paw_d405_handeye_eye_on_hand.yaml

用法：
  rosrun Eye_in_hand auto_sample.py
  或在 launch 中启动
"""

import sys
import rospy
import yaml
import os

import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty as EmptySrv

# easy_handeye 的服务类型（namespace 由 launch 中的 namespace_prefix 决定）
try:
    from easy_handeye.srv import TakeSample, ComputeCalibration, SaveCalibration
except ImportError:
    rospy.logfatal(
        "easy_handeye 的 srv 没找到。确认 easy_handeye 已经 clone 并编译："
        "cd ~/catkin_ws/src && git clone https://github.com/IFL-CAMP/easy_handeye.git && "
        "cd ~/catkin_ws && catkin_make"
    )
    sys.exit(1)


JOINT_ORDER = [
    "shoulder_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "wrist_yaw_joint",
]

# auto_poses.yaml 里 home_pose 的 key 对应 JOINT_ORDER
HOME_KEY_ORDER = [
    "shoulder_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "wrist_yaw",
]


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def wait_for_marker(topic, timeout):
    """等到 topic 上出现新消息（说明 aruco 检测到了）。"""
    try:
        rospy.wait_for_message(topic, PoseStamped, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def main():
    rospy.init_node("eye_in_hand_auto_sample", anonymous=False)

    # --- 读参数 ---
    cfg_path = rospy.get_param(
        "~config_path",
        os.path.join(
            os.environ.get("ROS_PACKAGE_PATH", "").split(":")[0],
            "Eye_in_hand/config/auto_poses.yaml",
        ),
    )
    handeye_ns = rospy.get_param("~handeye_namespace", "paw_d405_handeye_eye_on_hand")
    marker_topic = rospy.get_param("~marker_topic", "/aruco_single/pose")
    skip_missing_marker = rospy.get_param("~skip_missing_marker", True)
    auto_save = rospy.get_param("~auto_save", True)

    if not os.path.isfile(cfg_path):
        # fallback: rospkg
        try:
            import rospkg
            rp = rospkg.RosPack()
            cfg_path = os.path.join(
                rp.get_path("Eye_in_hand"), "config", "auto_poses.yaml"
            )
        except Exception as e:
            rospy.logfatal("找不到 auto_poses.yaml: %s", e)
            sys.exit(1)

    cfg = load_config(cfg_path)
    rospy.loginfo("加载位姿配置：%s", cfg_path)

    group_name = cfg.get("move_group_name", "manipulator")
    vel_scale = float(cfg.get("velocity_scaling", 0.2))
    acc_scale = float(cfg.get("acceleration_scaling", 0.2))
    settle_time = float(cfg.get("settle_time", 1.5))
    marker_timeout = float(cfg.get("aruco_check_timeout", 2.0))
    home = [float(cfg["home_pose"][k]) for k in HOME_KEY_ORDER]
    poses = cfg["poses"]

    # --- MoveIt 初始化 ---
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_max_velocity_scaling_factor(vel_scale)
    group.set_max_acceleration_scaling_factor(acc_scale)
    rospy.loginfo(
        "MoveGroup[%s] 规划组关节：%s",
        group_name,
        group.get_active_joints(),
    )

    # --- easy_handeye 服务 ---
    take_srv_name = "/{}/take_sample".format(handeye_ns)
    compute_srv_name = "/{}/compute_calibration".format(handeye_ns)
    save_srv_name = "/{}/save_calibration".format(handeye_ns)

    rospy.loginfo("等待 easy_handeye 服务：%s", take_srv_name)
    try:
        rospy.wait_for_service(take_srv_name, timeout=30.0)
    except rospy.ROSException:
        rospy.logfatal("easy_handeye 服务没启动，请先跑 eye_in_hand_calibrate.launch")
        sys.exit(1)

    take_sample = rospy.ServiceProxy(take_srv_name, TakeSample)
    compute_calibration = rospy.ServiceProxy(compute_srv_name, ComputeCalibration)
    save_calibration = rospy.ServiceProxy(save_srv_name, SaveCalibration)

    # --- 逐个位姿走一遍 ---
    total = len(poses)
    succeeded = 0
    failed_moves = []
    marker_missing = []

    for idx, p in enumerate(poses):
        name = p.get("name", "p%02d" % idx)
        delta = [float(x) for x in p["delta"]]
        target = [home[i] + delta[i] for i in range(6)]

        rospy.loginfo(
            "[%d/%d] 移动到位姿 %s  target=%s",
            idx + 1, total, name,
            ["%.3f" % x for x in target],
        )

        try:
            group.set_joint_value_target(target)
            ok = group.go(wait=True)
            group.stop()
        except Exception as e:
            rospy.logerr("MoveIt 规划异常：%s", e)
            ok = False

        if not ok:
            rospy.logwarn("位姿 %s 不可达，跳过", name)
            failed_moves.append(name)
            continue

        rospy.sleep(settle_time)

        if not wait_for_marker(marker_topic, marker_timeout):
            rospy.logwarn("位姿 %s：%s 无 marker 消息", name, marker_topic)
            marker_missing.append(name)
            if skip_missing_marker:
                continue

        try:
            resp = take_sample()
            n = len(resp.samples.hand_world_samples.transforms) \
                if hasattr(resp, "samples") else "?"
            rospy.loginfo("  采样成功，累计 %s 个", n)
            succeeded += 1
        except rospy.ServiceException as e:
            rospy.logerr("take_sample 失败：%s", e)

    # --- 回 home，安全姿态 ---
    rospy.loginfo("采样结束，回到 home 位姿")
    try:
        group.set_joint_value_target(home)
        group.go(wait=True)
        group.stop()
    except Exception:
        pass

    rospy.loginfo(
        "统计：成功 %d / 总 %d；MoveIt 不可达 %d；marker 丢失 %d",
        succeeded, total, len(failed_moves), len(marker_missing),
    )
    if failed_moves:
        rospy.logwarn("不可达位姿：%s", failed_moves)
    if marker_missing:
        rospy.logwarn("marker 丢失位姿：%s", marker_missing)

    if succeeded < 10:
        rospy.logerr("有效样本 <10，不做自动 compute。请手动在 rqt 里检查后再 compute")
        return

    if not auto_save:
        rospy.loginfo("auto_save=false，不自动求解。请在 rqt easy_handeye 里手动 Compute + Save")
        return

    rospy.loginfo("自动 Compute ...")
    try:
        compute_calibration()
    except rospy.ServiceException as e:
        rospy.logerr("compute_calibration 失败：%s", e)
        return

    rospy.loginfo("自动 Save ...")
    try:
        save_calibration()
        rospy.loginfo("结果保存到 ~/.ros/easy_handeye/%s.yaml", handeye_ns)
    except rospy.ServiceException as e:
        rospy.logerr("save_calibration 失败：%s", e)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
