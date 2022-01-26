#include "merge_maps_3d/g2o_optimizer.h"
#include "merge_maps_3d/converter.h"
#include "merge_maps_3d/edge_se3_priorvec.hpp"

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace g2o
{
G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
}

merge_maps_3d::G2oOptimizer::G2oOptimizer()
  : last_num_edges_ {0}
{
  graph_.reset(new g2o::SparseOptimizer());
  g2o::OptimizationAlgorithmProperty solver_property;
  graph_->setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_var", solver_property));
}

merge_maps_3d::G2oOptimizer::~G2oOptimizer()
{
  graph_.reset();
}

bool
merge_maps_3d::G2oOptimizer::addNode(Eigen::Matrix4f estimate, int id, bool fixed)
{
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(id);
  vertex->setEstimate(toIsometry(estimate));
  vertex->setFixed(fixed);
  graph_->addVertex(vertex);
  node_lookup_[id] = vertex;
  return true;
}

bool
merge_maps_3d::G2oOptimizer::addEdge(int id_to, int id_from, Eigen::Matrix4f connection, Eigen::MatrixXd information)
{
  g2o::EdgeSE3* edge(new g2o::EdgeSE3());
  edge->setMeasurement(toIsometry(connection));
  edge->setInformation(information);
  edge->vertices()[0] = node_lookup_[id_from];
  edge->vertices()[1] = node_lookup_[id_to];
  graph_->addEdge(edge);
  return true;
}

bool
merge_maps_3d::G2oOptimizer::addUnaryEdge(int id,
                                          const Eigen::Vector3d direction,
                                          const Eigen::Vector3d measurement,
                                          const Eigen::MatrixXd information)
{
  g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
  Eigen::Matrix<double, 6, 1> vec_measurement;
  vec_measurement.head<3>() = direction;
  vec_measurement.tail<3>() = measurement;
  edge->setMeasurement(vec_measurement);
  edge->setInformation(information);
  edge->vertices()[0] = node_lookup_[id];
  graph_->addEdge(edge);
  return true;
}

bool
merge_maps_3d::G2oOptimizer::optimize()
{
  if (graph_->edges().size() - last_num_edges_ < 10)
  {
    return true;
  }
  last_num_edges_ = graph_->edges().size();
  graph_->initializeOptimization();
  graph_->setVerbose(true);
  double before = graph_->chi2();
  graph_->optimize(5);
  std::cout << "Before " << before << " after " << graph_->chi2() << std::endl;
  return true;
}

std::map<int, Eigen::Matrix4f>
merge_maps_3d::G2oOptimizer::retrieveCorrected()
{
  std::map<int, Eigen::Matrix4f> data;
  for (const auto node : graph_->vertices())
  {
    Eigen::Isometry3d pose = dynamic_cast<g2o::VertexSE3*>(node.second)->estimate();
    data[node.first] = toMatrix(pose);
  }
  return data;
}
