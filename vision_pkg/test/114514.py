#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class PointCloudSubscriber:
    def __init__(self):
        self.paw_sub = rospy.Subscriber(
            "/paw_camera/depth/color/points", PointCloud2, self.paw_cloud_cb, queue_size=1
        )
        self.behind_sub = rospy.Subscriber(
            "/behind_camera/depth/color/points", PointCloud2, self.behind_cloud_cb, queue_size=1
        )

    def paw_cloud_cb(self, msg):
        # 将 PointCloud2 转为可迭代的点列表 (x, y, z, rgb)
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        rospy.loginfo("Paw cloud: %d points", len(cloud_points))

        # 在这里做点云处理（滤波、分割等）

    def behind_cloud_cb(self, msg):
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        rospy.loginfo("Behind cloud: %d points", len(cloud_points))


if __name__ == "__main__":
    rospy.init_node("pointcloud_subscriber")
    node = PointCloudSubscriber()
    rospy.spin()
