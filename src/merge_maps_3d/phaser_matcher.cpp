#include "merge_maps_3d/phaser_matcher.h"
#include "merge_maps_3d/converter.h"
#include <phaser/model/point-cloud.h>

merge_maps_3d::PhaserMatcher::PhaserMatcher()
{
  controller_.reset(new phaser_core::CloudController("sph-opt"));
}

merge_maps_3d::PhaserMatcher::~PhaserMatcher()
{
  controller_.reset();
}

std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
merge_maps_3d::PhaserMatcher::match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess)
{
  model::PointCloudPtr orig_model(new model::PointCloud(orig));
  model::PointCloudPtr addition_model(new model::PointCloud(addition));
  auto result = controller_->registerPointCloud(orig_model, addition_model);
  if (!result.foundSolution())
  {
    return std::nullopt;
  }

  auto cloud_registered = result.getRegisteredCloud();
  auto transform = result.getStateAsVec();
  Eigen::Matrix4f mat = toMatrix(transform);
  Cloud::Ptr cloud = cloud_registered->getRawCloud();
  return std::make_pair(mat, cloud);
}
