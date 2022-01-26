#ifndef MERGE_MAPS_3D_GICP_MERGER_H_
#define MERGE_MAPS_3D_GICP_MERGER_H_

#include "merge_maps_3d/cloud_merger.h"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/filters/voxel_grid.h>

namespace merge_maps_3d
{
class GicpMatcher final : public CloudMerger
{
public:
  GicpMatcher();
  ~GicpMatcher() override;

  std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
  match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess = std::nullopt) override;

private:
  std::shared_ptr<fast_gicp::FastGICP<Point, Point>> matcher_;

  std::shared_ptr<pcl::Filter<Point>> downsample_filter_;

  Cloud::Ptr downsample(Cloud::Ptr cloud);
};
} // namespace merge_maps_3d

#endif
