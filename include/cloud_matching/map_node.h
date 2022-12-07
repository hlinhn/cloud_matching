#ifndef CLOUD_MATCHING_MAP_NODE_H_
#define CLOUD_MATCHING_MAP_NODE_H_

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;

namespace cloud_matching
{
class MapNode
{
  static int node_id_;

public:
  MapNode() {};
  MapNode(Eigen::Matrix4f position, Cloud::Ptr cloud);
  ~MapNode();
  int id_;
  Eigen::Matrix4f position_;
  Eigen::Matrix4f unoptimized_position_;
  Cloud::Ptr cloud_;
  Point top_right;
  Point bottom_left;
  Point gravity_center;
  bool optimized_;
};
} // namespace cloud_matching

#endif
