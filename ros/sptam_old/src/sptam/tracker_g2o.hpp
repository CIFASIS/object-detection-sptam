/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include "Match.hpp"
#include "CameraPose.hpp"
#include "Measurement.hpp"
#include "g2o_driver.hpp"

#include <g2o/core/sparse_optimizer.h>

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class tracker_g2o
{
  public:

    /**
     * @brief This should be called for every frame of incoming stereo images.
     * This will use an estimate of the current camera pose and will try
     * to adjust it to the current map by minimizing reprojection errors.
     */
    CameraPose RefineCameraPose(
      const CameraPose& estimatedCameraPose,
      const StereoFrame::RectifiedCameraParameters& rectified_camera_parameters,
      const std::list<Match>& measurements
    );

    class not_enough_points : public std::runtime_error
    {
      public:
        not_enough_points()
          : std::runtime_error("Not enough points for tracking.") {}
    };

  protected:

  // G2O optimizer

    G2ODriver g2o_driver_;

  private:

  // helper functions

    Eigen::Matrix6d GetPoseCovariance(g2o::HyperGraph::Vertex& pose_vertex);
};
