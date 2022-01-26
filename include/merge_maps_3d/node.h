#ifndef MERGE_MAPS_3D_NODE_H_
#define MERGE_MAPS_3D_NODE_H_

#include "merge_maps_3d/global_map_maker.h"
#include "merge_maps_3d/local_map_maker.h"

#include <mutex>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

namespace merge_maps_3d
{
bool isFinite(Point p);
class Node
{
public:
  Node(ros::NodeHandle& node_handle);

private:
  LocalMapMaker local_map_maker_;
  GlobalMapMaker global_map_maker_;

  bool added_first_node_;
  bool got_first_imu_;

  std::optional<Eigen::Matrix4f> current_odom_position_;
  std::optional<Eigen::Vector3d> current_imu_data_;
  std::optional<Eigen::Vector3d> last_imu_data_;
  std::optional<Eigen::Matrix4f> last_odom_position_;

  Eigen::Matrix4f current_lidar_odom_position_;
  Eigen::Matrix4f last_lidar_odom_position_;
  Eigen::Matrix4f last_local_position_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher visualization_pub_;
  ros::Publisher loop_closure_viz_pub_;
  ros::Timer global_map_timer_;
  ros::ServiceServer save_map_server_;

  std::mutex guard_odom_;
  std::mutex guard_imu_;

  Cloud::Ptr filter(Cloud::Ptr cloud);
  std::optional<Eigen::Matrix4f> initialGuess(std::optional<Eigen::Matrix4f> current_odom_position);
  bool sufficient_distance(std::optional<Eigen::Matrix4f> current_position);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry& odom);
  void imuCallback(const sensor_msgs::Imu& imu);
  void globalMapCallback(const ros::TimerEvent& e);
  bool saveMapCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

} // namespace merge_maps_3d

#endif
