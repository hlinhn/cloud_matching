#include "merge_maps_3d/global_map_maker.h"
#include "merge_maps_3d/converter.h"
#include "merge_maps_3d/g2o_optimizer.h"
#include "merge_maps_3d/gicp_matcher.h"
#include "merge_maps_3d/phaser_matcher.h"
#include <pcl/io/pcd_io.h>

void
print_mat(Eigen::Matrix4f mat)
{
  auto rpy = toRPY(mat);
  for (unsigned int i = 0; i < 3; i++)
  {
    std::cout << rpy(i) << " ";
  }
  std::cout << std::endl;
  for (unsigned int i = 0; i < 3; i++)
  {
    std::cout << mat(i, 3) << " ";
  }
  std::cout << std::endl;
}

merge_maps_3d::GlobalMapMaker::GlobalMapMaker()
{
  matcher_.reset(new NdtMatcher());
  optimizer_.reset(new G2oOptimizer());
}

// void
// merge_maps_3d::GlobalMapMaker::setConfig(Config config)
// {
//   config_ = config;
// }

Eigen::MatrixXd
getConstantInformationMatrix()
{
  Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
  information.topLeftCorner(3, 3).array() /= 0.5;
  information.bottomRightCorner(3, 3).array() /= 0.1;
  return information;
}

Eigen::MatrixXd
getConstantInformationMatrixLoop()
{
  Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
  information.topLeftCorner(3, 3).array() /= 0.1;
  information.bottomRightCorner(3, 3).array() /= 0.05;
  return information;
}

Eigen::MatrixXd
getImuInformationMatrix()
{
  Eigen::MatrixXd information = Eigen::MatrixXd::Identity(3, 3) / 10.0;
  return information;
}

void
merge_maps_3d::GlobalMapMaker::addNode(const merge_maps_3d::MapNode& map_node, bool fixed)
{
  nodes_[map_node.id_] = map_node;
  std::cout << "Adding edge from " << map_node.id_ << " to " << map_node.id_ - 1 << std::endl;
  const std::lock_guard<std::mutex> lock(unprocessed_mutex_);
  unprocessed_nodes_.push_back(map_node.id_);

  if (fixed)
  {
    optimizer_->addNode(map_node.position_, map_node.id_, true);
    return;
  }
  else
  {
    optimizer_->addNode(map_node.position_, map_node.id_);
    auto information_matrix = getConstantInformationMatrix();
    optimizer_->addEdge(map_node.id_,
                        map_node.id_ - 1,
                        nodes_[map_node.id_ - 1].position_.inverse() * nodes_[map_node.id_].position_,
                        information_matrix);
  }
}

void
merge_maps_3d::GlobalMapMaker::addImuEdge(const int node_id, std::optional<Eigen::Vector3d> imu_data)
{
  if (!imu_data)
  {
    return;
  }
  auto information_matrix = getImuInformationMatrix();
  optimizer_->addUnaryEdge(node_id, Eigen::Vector3d(0, 0.0, -1.0), imu_data.value(), information_matrix);
}

// double
// merge_maps_3d::GlobalMapMaker::calculateIOU(int test_node, int propose_node)
// {

// }

std::vector<merge_maps_3d::Matching>
merge_maps_3d::GlobalMapMaker::checkLoopClosure(const int node_id)
{
  std::cout << "In loop closure check, checking " << nodes_.size() << std::endl;
  auto node_position = nodes_[node_id].position_.inverse();

  std::vector<std::pair<int, int>> candidates;
  for (const auto data : nodes_)
  {
    if (node_id < data.first + 5)
    {
      break;
    }
    Eigen::Vector4f center_check;
    center_check(0) = data.second.gravity_center.x;
    center_check(1) = data.second.gravity_center.y;
    center_check(2) = 0;
    center_check(3) = 1;
    auto check_position = data.second.position_;
    Eigen::Vector4f check_in_original = node_position * check_position * center_check;
    // float distance = (node_position * check_position).block<3, 1>(0, 3).norm();
    Eigen::Vector2f center_diff;
    center_diff(0) = check_in_original(0) - nodes_[node_id].gravity_center.x;
    center_diff(1) = check_in_original(1) - nodes_[node_id].gravity_center.y;
    float distance = center_diff.norm();
    if (distance > 6.0) // config_.max_loop_closure_check_distance)
    {
      continue;
    }
    candidates.push_back(std::make_pair(node_id, data.second.id_));
  }

  std::vector<Matching> filtered_candidates;
  for (const auto pair : candidates)
  {
    if (auto connection = checkPair(pair))
    {
      std::cout << "Adding " << pair.first << " " << pair.second << std::endl;
      loop_closure_links_.push_back(pair);

      auto information_matrix = getConstantInformationMatrixLoop();
      optimizer_->addEdge(pair.second, pair.first, connection.value(), information_matrix);
      filtered_candidates.push_back(Matching(pair, connection.value()));
    }
  }
  return filtered_candidates;
}

std::optional<Eigen::Matrix4f>
merge_maps_3d::GlobalMapMaker::checkPair(std::pair<int, int> candidate)
{
  auto proposed_node = nodes_[candidate.first];
  auto checking_node = nodes_[candidate.second];
  Cloud::Ptr checking_cloud(new Cloud());
  *checking_cloud += *checking_node.cloud_;
  // auto next_node = nodes_[candidate.second + 1];
  // Cloud::Ptr next_cloud(new Cloud());
  // pcl::transformPointCloud(*next_node.cloud_, *next_cloud, checking_node.position_.inverse() * next_node.position_);
  // *checking_cloud += *next_cloud;
  // if (candidate.second > 1)
  // {
  //   auto prev_node = nodes_[candidate.second - 1];
  //   Cloud::Ptr prev_cloud(new Cloud());
  //   pcl::transformPointCloud(*prev_node.cloud_, *prev_cloud, checking_node.position_.inverse() *
  //   prev_node.position_); *checking_cloud += *prev_cloud;
  // }
  Eigen::Matrix4f initial_guess = proposed_node.position_.inverse() * checking_node.position_;
  auto matching_data = matcher_->match(proposed_node.cloud_, checking_cloud, initial_guess);
  if (!matching_data)
  {
    return std::nullopt;
  }
  std::string name;
  name += std::to_string(candidate.first);
  name += std::string("_");
  name += std::to_string(candidate.second);
  name += std::string(".pcd");
  pcl::io::savePCDFileBinary(name, *matching_data.value().second);

  return matching_data.value().first;
}

std::vector<int>
merge_maps_3d::GlobalMapMaker::getUnprocessedNode()
{
  const std::lock_guard<std::mutex> lock(unprocessed_mutex_);
  return unprocessed_nodes_;
}

void
merge_maps_3d::GlobalMapMaker::removeProcessedNode(std::vector<int> processed)
{
  const std::lock_guard<std::mutex> lock(unprocessed_mutex_);
  unprocessed_nodes_.erase(unprocessed_nodes_.begin(), unprocessed_nodes_.begin() + processed.size());
}

Cloud::Ptr
merge_maps_3d::GlobalMapMaker::createMap()
{
  const std::lock_guard<std::mutex> lock(node_mutex_);
  Cloud::Ptr all_cloud(new Cloud());

  for (auto node_data : nodes_)
  {
    Cloud::Ptr transformed(new Cloud());
    pcl::transformPointCloud(*node_data.second.cloud_, *transformed, node_data.second.position_);
    // std::cout << "ID " << node_data.first << "\n";
    // print_mat(node_data.second.position_);
    // std::string name("");
    // name += std::to_string(node_data.first);
    // name += std::string(".pcd");
    // pcl::io::savePCDFileBinary(name, *node_data.second.cloud_);
    *all_cloud += *transformed;
  }
  return all_cloud;
}

void
merge_maps_3d::GlobalMapMaker::optimizeAndUpdate()
{
  const std::lock_guard<std::mutex> lock(node_mutex_);
  std::cout << "==========BEFORE===================\n";
  for (auto node_data : nodes_)
  {
    std::cout << node_data.first << ": ";
    print_mat(node_data.second.position_);
  }
  optimizer_->optimize();

  auto updated_info = optimizer_->retrieveCorrected();
  for (auto data : updated_info)
  {
    nodes_[data.first].position_ = data.second;
  }

  std::cout << "==========AFTER=====================\n";
  for (auto node_data : nodes_)
  {
    std::cout << node_data.first << ": ";
    print_mat(node_data.second.position_);
  }
}

std::vector<Eigen::Matrix4f>
merge_maps_3d::GlobalMapMaker::getUpdatedPosition()
{
  std::vector<Eigen::Matrix4f> collect;
  for (auto node_data : nodes_)
  {
    collect.push_back(node_data.second.position_);
  }
  return collect;
}

std::vector<std::pair<Eigen::Matrix4f, Eigen::Matrix4f>>
merge_maps_3d::GlobalMapMaker::getLoopClosureLinks()
{
  std::vector<std::pair<Eigen::Matrix4f, Eigen::Matrix4f>> links;
  for (auto pair : loop_closure_links_)
  {
    links.push_back(std::make_pair(nodes_[pair.first].position_, nodes_[pair.second].position_));
  }
  return links;
}
