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

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif

#include "../sptam/utils/projective_math.hpp"

// TODO call RectifiedCameraParameters
class CameraParameters
{
  public:

    CameraParameters(const cv::Matx33d& intrinsic, size_t image_width, size_t image_height, double frustum_near_plane_distance, double frustum_far_plane_distance, double baseline)
      : intrinsic_( intrinsic ), image_width_( image_width ), image_height_( image_height )
      , horizontal_fov_( computeFOV( intrinsic(0, 0), image_width ) )
      , vertical_fov_( computeFOV( intrinsic(1, 1), image_height ) )
      , frustum_near_plane_distance_( frustum_near_plane_distance )
      , frustum_far_plane_distance_( frustum_far_plane_distance )
      , baseline_( baseline )
    {
      assert( 0 < image_width_ );
      assert( 0 < image_height_ );
      assert( 0 < frustum_near_plane_distance_ );
      assert( frustum_near_plane_distance_ < frustum_far_plane_distance_ );
      assert( 0 < baseline_ );
      assert( intrinsic(0, 0) == intrinsic(1, 1) );
    }

    inline const cv::Matx33d& intrinsic() const
    { return intrinsic_; }

    inline float focalLength() const
    { return intrinsic_(0, 0); }

    inline Eigen::Vector2d focalLengths() const
    { return Eigen::Vector2d( intrinsic_(0, 0), intrinsic_(1, 1) ); }

    inline Eigen::Vector2d principalPoint() const
    { return Eigen::Vector2d( intrinsic_(0, 2), intrinsic_(1, 2) ); }

    inline size_t imageWidth() const
    { return image_width_; }

    inline size_t imageHeight() const
    { return image_height_; }

    inline double horizontalFov() const
    { return horizontal_fov_; }

    inline double verticalFov() const
    { return vertical_fov_; }

    double frustumNearPlaneDistance() const
    { return frustum_near_plane_distance_; }

    double frustumFarPlaneDistance() const
    { return frustum_far_plane_distance_; }

    double baseline() const
    { return baseline_; }

  private:

    cv::Matx33d intrinsic_;

    size_t image_width_, image_height_;

    double horizontal_fov_, vertical_fov_;

    double frustum_near_plane_distance_, frustum_far_plane_distance_;

    double baseline_;

  // helper functions

    /**
     * Return the Field Of View angle for a dimention of the camera,
     * given the focal length and size of the image in that dimention.
     */
    inline double computeFOV(const double focal_length, const double image_size)
    {
      return 2 * atan(image_size / (2 * focal_length)) * 180 / M_PI;
    }

};
