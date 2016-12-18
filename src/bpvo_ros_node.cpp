#include <ros/ros.h>

#include "bpvo_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bpvo_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  BPVORos bpvo(nh, nh_private);

  ros::spin();

  return 0;
}
