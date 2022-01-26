#include "merge_maps_3d/node.h"
#include "merge_maps_3d/converter.h"
#include <cmath>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

int merge_maps_3d::MapNode::node_id_ = 0;

merge_maps_3d::Node::Node(ros::NodeHandle& node_handle)
  : local_map_maker_ {}
  , global_map_maker_ {}
  , added_first_node_ {false}
  , got_first_imu_ {false}
{
  current_lidar_odom_position_ = Eigen::Matrix4f::Identity();
  last_lidar_odom_position_ = Eigen::Matrix4f::Identity();
  last_local_position_ = Eigen::Matrix4f::Identity();

  cloud_sub_ = node_handle.subscribe("/point_cloud", 1, &Node::cloudCallback, this);
  odom_sub_ = node_handle.subscribe("/odom", 1, &Node::odomCallback, this);
  imu_sub_ = node_handle.subscribe("/imu", 1, &Node::imuCallback, this);
  cloud_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/local_cloud", 1);
  global_map_timer_ = node_handle.createTimer(ros::Duration(1.0), &Node::globalMapCallback, this);
  save_map_server_ = node_handle.advertiseService("save_map", &Node::saveMapCallback, this);
  visualization_pub_ = node_handle.advertise<geometry_msgs::PoseArray>("submaps", 1);
  loop_closure_viz_pub_ = node_handle.advertise<visualization_msgs::Marker>("loop_closure", 1);
}

bool
merge_maps_3d::isFinite(Point p)
{
  return !(std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z) || std::isnan(p.x) || std::isnan(p.y)
           || std::isnan(p.z));
}

bool
merge_maps_3d::Node::saveMapCallback(std_srvs::Empty::Request& /* request */, std_srvs::Empty::Response& /* response */)
{
  auto all_cloud = global_map_maker_.createMap();
  pcl::io::savePCDFileBinary("test_map.pcd", *all_cloud);
  return true;
}

void
merge_maps_3d::Node::imuCallback(const sensor_msgs::Imu& imu)
{
  const std::lock_guard<std::mutex> lock(guard_imu_);
  if (!got_first_imu_)
  {
    last_imu_data_ = toVector(imu);
    got_first_imu_ = true;
  }
  current_imu_data_ = toVector(imu);
}

void
merge_maps_3d::Node::odomCallback(const nav_msgs::Odometry& odom)
{
  const std::lock_guard<std::mutex> lock(guard_odom_);
  current_odom_position_ = toMatrix(odom);
}

Cloud::Ptr
merge_maps_3d::Node::filter(Cloud::Ptr cloud)
{
  Cloud::Ptr filtered(new Cloud());
  for (const auto p : *cloud)
  {
    if (isFinite(p))
    {
      filtered->push_back(p);
    }
  }
  return filtered;
}

std::optional<Eigen::Matrix4f>
merge_maps_3d::Node::initialGuess(std::optional<Eigen::Matrix4f> current_odom_position)
{
  if (!current_odom_position)
  {
    return std::nullopt;
  }
  if (!last_odom_position_)
  {
    return Eigen::Matrix4f::Identity();
  }
  return last_local_position_ * (last_odom_position_.value().inverse() * current_odom_position.value());
}

bool
merge_maps_3d::Node::sufficient_distance(std::optional<Eigen::Matrix4f> current_position)
{
  if (!last_odom_position_ || !current_position)
  {
    return true;
  }
  auto diff = last_odom_position_.value().inverse() * current_position.value();
  double distance_diff = diff.block<3, 1>(0, 3).norm();
  if (distance_diff < 0.3)
  {
    return false;
  }
  return true;
}

void
merge_maps_3d::Node::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::optional<Eigen::Matrix4f> current_odom_position_copy;
  {
    const std::lock_guard<std::mutex> lock(guard_odom_);
    current_odom_position_copy = current_odom_position_;
  }

  std::optional<Eigen::Vector3d> current_imu_data_copy;
  {
    const std::lock_guard<std::mutex> lock(guard_imu_);
    current_imu_data_copy = current_imu_data_;
  }

  if (!sufficient_distance(current_odom_position_copy))
  {
    return;
  }
  Cloud::Ptr cloud(new Cloud());
  pcl::fromROSMsg(*msg, *cloud);
  Cloud::Ptr filtered_cloud = filter(cloud);
  const auto initial_guess = initialGuess(current_odom_position_copy);
  // std::cout << "Initial guess ";
  // print_mat(initial_guess.value());
  auto current_lidar_odom = local_map_maker_.addCloud(filtered_cloud, initial_guess);

  if (!current_lidar_odom)
  {
    return;
  }

  // std::cout << "Fixed ";
  // print_mat(current_lidar_odom.value());

  last_local_position_ = current_lidar_odom.value();
  current_lidar_odom_position_ = last_lidar_odom_position_ * last_local_position_;
  last_odom_position_ = current_odom_position_copy;

  if (auto const node = local_map_maker_.nodeReady())
  {
    local_map_maker_.restart(current_lidar_odom_position_, filtered_cloud);
    if (!added_first_node_)
    {
      added_first_node_ = true;
      global_map_maker_.addNode(node.value(), true);
    }
    else
    {
      global_map_maker_.addNode(node.value(), false);
    }

    // off by one?
    global_map_maker_.addImuEdge(node.value().id_, last_imu_data_);
    last_imu_data_ = current_imu_data_copy;
    last_lidar_odom_position_ = current_lidar_odom_position_;
    last_local_position_ = Eigen::Matrix4f::Identity();
  }

  Cloud::Ptr current_cloud = local_map_maker_.getCloud();
  Cloud::Ptr current_cloud_at_odom(new Cloud());
  pcl::transformPointCloud(*current_cloud, *current_cloud_at_odom, last_lidar_odom_position_);
  sensor_msgs::PointCloud2 current_msg;
  pcl::toROSMsg(*current_cloud_at_odom, current_msg);
  current_msg.header.stamp = msg->header.stamp;
  current_msg.header.frame_id = "odom";
  cloud_pub_.publish(current_msg);
}

void
merge_maps_3d::Node::globalMapCallback(const ros::TimerEvent& e)
{
  auto processing = global_map_maker_.getUnprocessedNode();
  for (auto id : processing)
  {
    auto connections = global_map_maker_.checkLoopClosure(id);
  }
  global_map_maker_.removeProcessedNode(processing);
  global_map_maker_.optimizeAndUpdate();
  std::vector<Eigen::Matrix4f> submap_position = global_map_maker_.getUpdatedPosition();
  geometry_msgs::PoseArray visual;
  visual.header.stamp = e.current_real;
  visual.header.frame_id = "odom";
  for (auto pose : submap_position)
  {
    geometry_msgs::Pose p = toPose(pose);
    visual.poses.push_back(p);
  }
  visualization_pub_.publish(visual);

  visualization_msgs::Marker loop_closure_marker;
  loop_closure_marker.header = visual.header;
  loop_closure_marker.type = visualization_msgs::Marker::LINE_LIST;
  loop_closure_marker.action = visualization_msgs::Marker::MODIFY;
  loop_closure_marker.color.g = 1.0;
  loop_closure_marker.color.a = 1.0;
  loop_closure_marker.scale.x = 0.1;
  auto loop_closure_links = global_map_maker_.getLoopClosureLinks();
  for (auto pair : loop_closure_links)
  {
    geometry_msgs::Point p = toPoint(pair.first);
    loop_closure_marker.points.push_back(p);
    geometry_msgs::Point q = toPoint(pair.second);
    loop_closure_marker.points.push_back(q);
  }
  loop_closure_viz_pub_.publish(loop_closure_marker);
}
