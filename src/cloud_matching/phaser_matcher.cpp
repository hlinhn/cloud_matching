#include "cloud_matching/phaser_matcher.h"
#include "cloud_matching/converter.h"
#include <phaser/model/point-cloud.h>

cloud_matching::PhaserMatcher::PhaserMatcher()
{
  controller_.reset(new phaser_core::CloudController("sph-opt"));
}

cloud_matching::PhaserMatcher::~PhaserMatcher()
{
  controller_.reset();
}

std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
cloud_matching::PhaserMatcher::match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess)
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
