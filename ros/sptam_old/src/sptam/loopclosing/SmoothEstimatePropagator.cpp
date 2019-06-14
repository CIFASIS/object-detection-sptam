#include "SmoothEstimatePropagator.hpp"

#include <Eigen/Eigen>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

SmoothEstimatePropagator::SmoothEstimatePropagator(g2o::SparseOptimizer* g,
                         const double& maxDistance,
                         const double& maxEdgeCost)
  : g2o::EstimatePropagator(g), _smoothAction(&_adjacencyMap,maxDistance), _treeCost(g),
    _maxDistance(maxDistance), _maxEdgeCost(maxEdgeCost) {}

void SmoothEstimatePropagator::propagate(g2o::OptimizableGraph::Vertex* v)
{ g2o::EstimatePropagator::propagate(v, _treeCost, _smoothAction, _maxDistance, _maxEdgeCost); }

SmoothEstimatePropagator::SmoothPropagateAction::
  SmoothPropagateAction(g2o::EstimatePropagator::AdjacencyMap* adj, const double& max_distance)
  : adjacency(adj), maxDistance(max_distance){}

void SmoothEstimatePropagator::SmoothPropagateAction::
  operator()(g2o::OptimizableGraph::Edge* e_, const g2o::OptimizableGraph::VertexSet& from_, g2o::OptimizableGraph::Vertex* to_) const
{
  if (to_->fixed())
    return;

  // Static cast to SE3, this must be ensure beforehand using the propagator.
  g2o::VertexSE3* from = static_cast<g2o::VertexSE3*>(e_->vertex(0));
  g2o::VertexSE3* to = static_cast<g2o::VertexSE3*>(e_->vertex(1));
  g2o::EdgeSE3* e = static_cast<g2o::EdgeSE3*>(e_);

  if (from_.count(from) > 0){
    auto entry = adjacency->find(to);
    double distance = entry == adjacency->end() ? 0 : entry->second.distance(); // this shouldnt happen! "to" must be on the adjacency map
    to->setEstimate(exponencialInterpolation(to->estimate(), from->estimate() * e->measurement(), distance));
  }else{
    auto entry = adjacency->find(from);
    double distance = entry == adjacency->end() ? 0 : entry->second.distance(); // this shouldnt happen! "to" must be on the adjacency map
    from->setEstimate(exponencialInterpolation(from->estimate(), to->estimate() * e->measurement().inverse(), distance));
  }
}

Eigen::Isometry3d SmoothEstimatePropagator::SmoothPropagateAction::
  exponencialInterpolation(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to, double step) const
{
  Eigen::Isometry3d res;

  double maxdist = maxDistance-2;

  // step goes from 1 to maxDistance, we need x from 0 to 1 in a linear way.
  double x = 1 - ((maxdist - (step-1))/maxdist);
  // alpha in [0, inf) describes the explonential ramp "steepness"
  double alpha = 50;
  // exponential ramp from 0 to 1
  double exp_ramp = 1 - (x/(1+alpha*(1-x)));

  // using quaternion representation and slerp for interpolate between from and to isometry transformations
  res.linear() = (Eigen::Quaterniond(from.rotation()).slerp(exp_ramp, Eigen::Quaterniond(to.rotation()))).toRotationMatrix();
  res.translation() = (1 - exp_ramp) * from.translation() + exp_ramp * to.translation();

  return res;
}
