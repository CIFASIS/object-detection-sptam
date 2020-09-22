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
#include "Camera.hpp"

inline FrustumCulling computeFrustum(const CameraPose& pose, const CameraParameters& calibration)
{
  return FrustumCulling(
    pose.GetPosition(), pose.GetOrientationQuaternion(),
    calibration.horizontalFov(), calibration.verticalFov(),
    calibration.frustumNearPlaneDistance(), calibration.frustumFarPlaneDistance()
  );
}

inline cv::Matx34d computeTransformation(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation)
{
  // R = O'
  const Eigen::Matrix3d rotationMatrix = orientation.transpose();

  // t = -R * C where C is the camera position
  const Eigen::Vector3d translation = -rotationMatrix * position;

  return cv::Matx34d(
    rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), translation[0],
    rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), translation[1],
    rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), translation[2]
  );
}

Camera::Camera(const CameraPose& pose, const CameraParameters& calibration)
  : pose_( pose ), calibration_( calibration )
  , transformation_( computeTransformation( pose_.GetPosition(), pose_.GetOrientationMatrix() ) )
  , projection_( calibration_.intrinsic() * transformation_ )
  , frustum_( computeFrustum( pose, calibration ) )
{}

void Camera::UpdatePose(const CameraPose& newPose)
{
  pose_ = newPose;

  frustum_ = computeFrustum( pose_, calibration_ );
  transformation_ = computeTransformation( pose_.GetPosition(), pose_.GetOrientationMatrix() );
  projection_ = calibration_.intrinsic() * transformation_;
}

std::ostream& operator << ( std::ostream& os, const Camera& camera)
{
  return os << camera.GetPose();
}
