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

#include "CameraPose.hpp"
#include "Measurement.hpp"

namespace cv {
  typedef Matx<double, 2, 6> Matx26d;
}

class Tracker
{
  public:

    Tracker(
      const cv::Matx33d& intrinsicLeft,
			const cv::Matx33d& intrinsicRight,
      double stereo_baseline
    );

    /**
     * TrackFrame is the main working part of the tracker:
     * call this for every frame.
     * This will estimate the new pose of the system
     * and return a potential CameraPose.
     */
    CameraPose RefineCameraPose(
      const CameraPose& estimatedCameraPose,
      const std::vector<Measurement>& measurementsLeft, const std::vector<Measurement>& measurementsRight
    );

  protected:

  // Calibration

		cv::Matx33d intrinsicLeft_;
		cv::Matx33d intrinsicRight_;

    double stereo_baseline_;
};

cv::Matx26d ComputeJacobian(
  double fu, double fv,
  const cv::Point3d& pointInPreviousCameraFrame,
  const cv::Point3d& pointInRefinedCameraFrame
);

/**
 * Convert the vector mu = (mu1,mu2, mu3, mu4 ,mu5 ,mu6) to a SE3 Matrix.
 * This functions is the MuToSE3Mat
 */
cv::Matx44d ExponentialMap(const cv::Vec6d& mu);

