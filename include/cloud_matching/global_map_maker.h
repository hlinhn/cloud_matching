#ifndef CLOUD_MATCHING_GLOBAL_MAP_MAKER_H_
#define CLOUD_MATCHING_GLOBAL_MAP_MAKER_H_

#include "cloud_matching/CloudMatchingConfig.h"
#include "cloud_matching/cloud_merger.h"
#include "cloud_matching/map_node.h"
#include "cloud_matching/ndt_matcher.h"
#include "cloud_matching/optimizer.h"
#include <map>
#include <mutex>

namespace cloud_matching
{
struct Matching
{
  Matching(int first_id, int second_id, Eigen::Matrix4f position)
    : first_id(first_id)
    , second_id(second_id)
    , position(position)
  {
  }
  Matching(std::pair<int, int> pair, Eigen::Matrix4f position)
    : first_id(pair.first)
    , second_id(pair.second)
    , position(position)
  {
  }
  int first_id;
  int second_id;
  Eigen::Matrix4f position;
};

class GlobalMapMaker
{
public:
  GlobalMapMaker();
  void addNode(const MapNode& map_node, bool fixed);
  void addImuEdge(const int node_id, std::optional<Eigen::Vector3d> imu_data);

  std::optional<std::vector<Matching>> checkLoopClosure(const int node_id);
  std::vector<int> getUnprocessedNode();
  void removeProcessedNode(std::vector<int> processed);
  void optimizeAndUpdate();
  std::vector<Eigen::Matrix4f> getUpdatedPosition();
  std::vector<std::pair<Eigen::Matrix4f, Eigen::Matrix4f>> getLoopClosureLinks();
  Cloud::Ptr createMap();
  void setConfig(CloudMatchingConfig& config);

private:
  std::mutex node_mutex_;
  std::mutex matcher_mutex_;
  std::map<int, MapNode> nodes_;
  std::shared_ptr<CloudMerger> matcher_;
  std::shared_ptr<Optimizer> optimizer_;
  std::optional<Eigen::Matrix4f> checkPair(std::pair<int, int> candidate);
  void changeMatchingMethod(int type);
  std::mutex unprocessed_mutex_;
  std::vector<int> unprocessed_nodes_;
  std::vector<std::pair<int, int>> loop_closure_links_;
  CloudMatchingConfig config_;
};
} // namespace cloud_matching

#endif
