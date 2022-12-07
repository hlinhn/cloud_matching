#include "cloud_matching/gicp_matcher.h"

cloud_matching::GicpMatcher::GicpMatcher()
{
  matcher_.reset(new fast_gicp::FastGICP<Point, Point>());
  matcher_->setMaxCorrespondenceDistance(1.0);

  auto voxel_grid = new pcl::VoxelGrid<Point>();
  double voxel_size = 0.15;
  voxel_grid->setLeafSize(voxel_size, voxel_size, voxel_size);
  downsample_filter_.reset(voxel_grid);
}

cloud_matching::GicpMatcher::~GicpMatcher() {}

Cloud::Ptr
cloud_matching::GicpMatcher::downsample(Cloud::Ptr cloud)
{
  Cloud::Ptr filtered(new Cloud());
  downsample_filter_->setInputCloud(cloud);
  downsample_filter_->filter(*filtered);
  return filtered;
}

std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
cloud_matching::GicpMatcher::match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess)
{
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  if (initial_guess)
  {
    init_guess = initial_guess.value();
  }
  Cloud::Ptr transformed(new Cloud());

  auto orig_filtered = downsample(orig);
  auto addition_filtered = downsample(addition);
  matcher_->setInputTarget(orig_filtered);
  matcher_->setInputSource(addition_filtered);
  matcher_->align(*transformed, init_guess);
  Eigen::Matrix4f corrected = matcher_->getFinalTransformation();
  pcl::transformPointCloud(*addition, *transformed, corrected);
  *transformed += *orig;
  if (!matcher_->hasConverged()) // || matcher_->getFitnessScore() > 1.0)
  {
    return std::nullopt;
  }
  std::cout << matcher_->getFitnessScore() << std::endl;
  return std::make_pair(corrected, transformed);
}
