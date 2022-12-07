#include "cloud_matching/node.h"

#include <ros/console.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_matching");

  ros::NodeHandle node_handle("~");
  cloud_matching::Node cloud_matching_node(node_handle);

  ROS_INFO("Initialized");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}
