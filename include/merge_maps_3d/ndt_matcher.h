#ifndef MERGE_MAPS_3D_NDT_MERGER_H_
#define MERGE_MAPS_3D_NDT_MERGER_H_

#include "merge_maps_3d/cloud_merger.h"
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

namespace merge_maps_3d
{
class NdtMatcher final : public CloudMerger
{
public:
  NdtMatcher();
  ~NdtMatcher() override;

  std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
  match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess = std::nullopt) override;

private:
  std::shared_ptr<pclomp::NormalDistributionsTransform<Point, Point>> matcher_;
  std::shared_ptr<pcl::Filter<Point>> downsample_filter_;

  Cloud::Ptr downsample(Cloud::Ptr cloud);
  bool isCompatible();
};
} // namespace merge_maps_3d

#endif
