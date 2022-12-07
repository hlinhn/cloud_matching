#ifndef CLOUD_MATCHING_G2O_OPTIMIZER_H_
#define CLOUD_MATCHING_G2O_OPTIMIZER_H_

#include "cloud_matching/optimizer.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <memory>

namespace cloud_matching
{
class G2oOptimizer final : public Optimizer
{
public:
  G2oOptimizer();
  ~G2oOptimizer() override;

  bool addNode(Eigen::Matrix4f estimate, int id, bool fixed) override;
  bool updateNode(Eigen::Matrix4f estimate, int id) override;
  bool addUnaryEdge(int id,
                    const Eigen::Vector3d direction,
                    const Eigen::Vector3d measurement,
                    const Eigen::MatrixXd information) override;
  bool addEdge(int id_to, int id_from, Eigen::Matrix4f connection, Eigen::MatrixXd information) override;
  bool optimize() override;
  std::map<int, Eigen::Matrix4f> retrieveCorrected() override;

private:
  std::unique_ptr<g2o::SparseOptimizer> graph_;
  std::map<int, g2o::VertexSE3*> node_lookup_;
  int last_num_edges_;
};
} // namespace cloud_matching

#endif
