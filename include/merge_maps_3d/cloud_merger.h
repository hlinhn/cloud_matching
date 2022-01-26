#ifndef MERGE_MAPS_3D_CLOUD_MERGER_H_
#define MERGE_MAPS_3D_CLOUD_MERGER_H_

#include <Eigen/Core>
#include <optional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Cloud;

namespace merge_maps_3d
{
class CloudMerger
{
public:
  CloudMerger() = default;
  virtual ~CloudMerger() = default;
  virtual std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>> match(Cloud::Ptr orig,
                                                                      Cloud::Ptr addition,
                                                                      std::optional<Eigen::Matrix4f> initial_guess) = 0;
};
} // namespace merge_maps_3d

#endif
