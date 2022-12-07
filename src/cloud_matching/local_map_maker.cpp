#include "cloud_matching/local_map_maker.h"
#include "cloud_matching/gicp_matcher.h"

cloud_matching::LocalMapMaker::LocalMapMaker()
  : first_cloud_(true)
{
  current_cloud_.reset(new Cloud());
  matcher_.reset(new GicpMatcher());
  current_position_ = Eigen::Matrix4f::Identity();
  origin_ = Eigen::Matrix4f::Identity();
}

std::optional<Eigen::Matrix4f>
cloud_matching::LocalMapMaker::addCloud(Cloud::Ptr cloud, std::optional<Eigen::Matrix4f> initial)
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
cloud_matching::LocalMapMaker::restart(Eigen::Matrix4f new_origin, Cloud::Ptr seed_cloud)
{
  // first_cloud_ = true;
  current_cloud_.reset(new Cloud(*seed_cloud));
  current_position_ = Eigen::Matrix4f::Identity();
  origin_ = new_origin;
}

Cloud::Ptr
cloud_matching::LocalMapMaker::getCloud()
{
  return current_cloud_;
}

std::optional<cloud_matching::MapNode>
cloud_matching::LocalMapMaker::nodeReady()
{
  if ((current_position_.block<2, 1>(0, 3).norm() < 2.0) && current_cloud_->size() < 700000)
  {
    return std::nullopt;
  }
  return MapNode(origin_, current_cloud_);
}
