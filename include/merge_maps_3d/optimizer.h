#ifndef MERGE_MAPS_3D_OPTIMIZER_H_
#define MERGE_MAPS_3D_OPTIMIZER_H_

#include <Eigen/Core>
#include <map>

namespace merge_maps_3d
{
class Optimizer
{
public:
  Optimizer() = default;
  virtual ~Optimizer() = default;
  virtual bool addNode(Eigen::Matrix4f estimate, int id, bool fixed = false) = 0;
  virtual bool addEdge(int id_to, int id_from, Eigen::Matrix4f connection, Eigen::MatrixXd information) = 0;
  virtual bool addUnaryEdge(int id,
                            const Eigen::Vector3d direction,
                            const Eigen::Vector3d measurement,
                            const Eigen::MatrixXd information) = 0;
  virtual bool optimize() = 0;
  virtual std::map<int, Eigen::Matrix4f> retrieveCorrected() = 0;
};
} // namespace merge_maps_3d

#endif
