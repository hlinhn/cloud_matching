#include "merge_maps_3d/map_node.h"

merge_maps_3d::MapNode::MapNode(Eigen::Matrix4f position, Cloud::Ptr cloud)
  : position_(position)
{
  cloud_.reset(new Cloud(*cloud));
  node_id_++;
  id_ = node_id_;
  if (cloud->size() < 1)
  {
    return;
  }
  double xmin = cloud->points[0].x;
  double xmax = xmin;
  double ymin = cloud->points[0].y;
  double ymax = ymin;
  double xsum = 0;
  double ysum = 0;

  for (const auto p : *cloud)
  {
    xsum += p.x;
    ysum += p.y;
    if (p.x > xmax)
    {
      xmax = p.x;
    }
    if (p.x < xmin)
    {
      xmin = p.x;
    }
    if (p.y > ymax)
    {
      ymax = p.y;
    }
    if (p.y < ymin)
    {
      ymin = p.y;
    }
  }
  bottom_left.x = xmin;
  bottom_left.y = ymin;
  top_right.x = xmax;
  top_right.y = ymax;
  gravity_center.x = xsum / cloud->size();
  gravity_center.y = ysum / cloud->size();
}

merge_maps_3d::MapNode::~MapNode()
{
  cloud_.reset();
}
