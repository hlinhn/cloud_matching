#ifndef MERGE_MAPS_3D_LOCAL_MAP_MAKER_H_
#define MERGE_MAPS_3D_LOCAL_MAP_MAKER_H_

#include "merge_maps_3d/cloud_merger.h"
#include "merge_maps_3d/map_node.h"
#include "merge_maps_3d/ndt_matcher.h"
#include <optional>

namespace merge_maps_3d
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

} // namespace merge_maps_3d

#endif
