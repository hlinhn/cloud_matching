#include "merge_maps_3d/local_map_maker.h"
#include "merge_maps_3d/gicp_matcher.h"

merge_maps_3d::LocalMapMaker::LocalMapMaker()
  : first_cloud_(true)
{
  current_cloud_.reset(new Cloud());
  matcher_.reset(new GicpMatcher());
  current_position_ = Eigen::Matrix4f::Identity();
  origin_ = Eigen::Matrix4f::Identity();
}

std::optional<Eigen::Matrix4f>
merge_maps_3d::LocalMapMaker::addCloud(Cloud::Ptr cloud, std::optional<Eigen::Matrix4f> initial)
{
  if (first_cloud_)
  {
    *current_cloud_ += *cloud;
    // if (!initial)
    // {
    //   origin_ = origin_ * initial.value();
    // }
    first_cloud_ = false;
    return Eigen::Matrix4f::Identity();
  }

  if (!initial)
  {
    initial = current_position_;
  }

  auto matching_data = matcher_->match(current_cloud_, cloud, initial);
  if (matching_data)
  {
    *current_cloud_ = *matching_data.value().second;
    current_position_ = matching_data.value().first;
    return current_position_;
  }
  return std::nullopt;
}

void
merge_maps_3d::LocalMapMaker::restart(Eigen::Matrix4f new_origin, Cloud::Ptr seed_cloud)
{
  // first_cloud_ = true;
  current_cloud_.reset(new Cloud(*seed_cloud));
  current_position_ = Eigen::Matrix4f::Identity();
  origin_ = new_origin;
}

Cloud::Ptr
merge_maps_3d::LocalMapMaker::getCloud()
{
  return current_cloud_;
}

std::optional<merge_maps_3d::MapNode>
merge_maps_3d::LocalMapMaker::nodeReady()
{
  if ((current_position_.block<3, 1>(0, 3).norm() < 3.5) && current_cloud_->size() < 700000)
  {
    return std::nullopt;
  }
  return MapNode(origin_, current_cloud_);
}
