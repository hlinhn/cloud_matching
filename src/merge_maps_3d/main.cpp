#include "merge_maps_3d/node.h"

#include <ros/console.h>
#include <ros/node_handle.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_maps_3d");

  ros::NodeHandle node_handle("~");
  merge_maps_3d::Node merge_maps_3d_node(node_handle);

  ROS_INFO("Initialized");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}
