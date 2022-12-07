#ifndef CLOUD_MATCHING_LOCAL_MAP_MAKER_H_
#define CLOUD_MATCHING_LOCAL_MAP_MAKER_H_

#include "cloud_matching/cloud_merger.h"
#include "cloud_matching/map_node.h"
#include "cloud_matching/ndt_matcher.h"
#include <optional>

namespace cloud_matching
{
class LocalMapMaker
{
public:
  LocalMapMaker();
  std::optional<Eigen::Matrix4f> addCloud(Cloud::Ptr cloud, std::optional<Eigen::Matrix4f> initial = std::nullopt);
  void restart(Eigen::Matrix4f, Cloud::Ptr seed_cloud);
  std::optional<MapNode> nodeReady();
  Cloud::Ptr getCloud();

private:
  Cloud::Ptr current_cloud_;
  bool first_cloud_;
  std::shared_ptr<CloudMerger> matcher_;
  Eigen::Matrix4f current_position_;
  Eigen::Matrix4f origin_;
};

} // namespace cloud_matching

#endif
