#ifndef MERGE_MAPS_3D_G2O_OPTIMIZER_H_
#define MERGE_MAPS_3D_G2O_OPTIMIZER_H_

#include "merge_maps_3d/optimizer.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <memory>

namespace merge_maps_3d
{
class G2oOptimizer final : public Optimizer
{
public:
  G2oOptimizer();
  ~G2oOptimizer() override;

  bool addNode(Eigen::Matrix4f estimate, int id, bool fixed) override;
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
} // namespace merge_maps_3d

#endif
