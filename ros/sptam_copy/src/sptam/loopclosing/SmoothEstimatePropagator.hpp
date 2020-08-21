#ifndef __SMOOTH_ESTIMATE_PROPAGATOR_HPP__
#define __SMOOTH_ESTIMATE_PROPAGATOR_HPP__

#include <Eigen/Eigen>

// G2O
#include <g2o/config.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/estimate_propagator.h>

/*
 * SmoothEstimatePropagator propagates edge information relaxing it while moving away of fixed vertices.
 * Uses an exponential function for this, so a vertex near fixed vertices will be moved all the way accordingly
 * with edge restriction meanwhile a vertex far away will remain in the same place.
 *
 * Note that this propagator only works with VertexSE3 and EdgeSE3.
 */

class SmoothEstimatePropagator : g2o::EstimatePropagator
{
  public:

    SmoothEstimatePropagator(g2o::SparseOptimizer* g,
                             const double& maxDistance=std::numeric_limits<double>::max(),
                             const double& maxEdgeCost=std::numeric_limits<double>::max());

    void propagate(g2o::OptimizableGraph::Vertex* v);

  private:

    struct SmoothPropagateAction : g2o::EstimatePropagator::PropagateAction {
      public:
        SmoothPropagateAction(g2o::EstimatePropagator::AdjacencyMap* adj, const double& max_distance);

        void operator()(g2o::OptimizableGraph::Edge* e_, const g2o::OptimizableGraph::VertexSet& from_, g2o::OptimizableGraph::Vertex* to_) const;

      private:
        g2o::EstimatePropagator::AdjacencyMap* adjacency;
        double maxDistance;

        Eigen::Isometry3d exponencialInterpolation(const Eigen::Isometry3d& from, const Eigen::Isometry3d& to, double step) const;
    };

    SmoothPropagateAction _smoothAction;
    g2o::EstimatePropagator::PropagateCost _treeCost;
    double _maxDistance, _maxEdgeCost;
};

#endif //__SMOOTH_ESTIMATE_PROPAGATOR_HPP__
