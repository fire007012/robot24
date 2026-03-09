#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudSubscriber {
public:
    PointCloudSubscriber() {
        ros::NodeHandle nh;
        paw_sub_ = nh.subscribe("/paw_camera/depth/color/points", 1,
                                &PointCloudSubscriber::pawCloudCb, this);
        behind_sub_ = nh.subscribe("/behind_camera/depth/color/points", 1,
                                   &PointCloudSubscriber::behindCloudCb, this);
    }

private:
    void pawCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        ROS_INFO("Paw cloud: %lu points", cloud->points.size());
        
        // 在这里做点云处理（滤波、分割等）
    }

    void behindCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        ROS_INFO("Behind cloud: %lu points", cloud->points.size());
    }

    ros::Subscriber paw_sub_;
    ros::Subscriber behind_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_subscriber");
    PointCloudSubscriber node;
    ros::spin();
    return 0;
}
