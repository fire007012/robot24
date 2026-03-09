#include <ros/ros.h>

#include "arm_traj_splitter/SplitterNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_traj_splitter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    arm_traj_splitter::SplitterNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("[arm_traj_splitter] startup failed: %s", e.what());
    return 1;
  }

  return 0;
}
