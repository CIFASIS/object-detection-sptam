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

#include <set>
#include <iostream>
#include <eigen3/Eigen/Geometry>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif


class ObjectPoint
{
  public:

    ObjectPoint(const Eigen::Vector3d& position);

    ObjectPoint(const ObjectPoint& objPoint);

    inline Eigen::Vector3d GetPosition() const
    {
      boost::shared_lock<boost::shared_mutex> lock(opoint_mutex_);
      return position_;
    }

    inline void updatePosition(const Eigen::Vector3d& new_position)
    {
      boost::unique_lock<boost::shared_mutex> lock(opoint_mutex_);
      position_ = new_position;
    }

     
     /**
     * @brief TODO give the color a meaning
     *
     * @return
     *   (r,g,b) components are in the range [0, 255).
     */
    inline cv::Vec3d getColor() const
    { 
      boost::shared_lock<boost::shared_mutex> lock(opoint_mutex_);
      return color_ ;
    }

    inline void updateColor(const cv::Vec3b& new_color)
    {
      boost::shared_lock<boost::shared_mutex> lock(opoint_mutex_);
      color_ = new_color ;
    } 

  private:

    mutable boost::shared_mutex opoint_mutex_;

    // position in world coordinates.
    Eigen::Vector3d position_;

    cv::Vec3b color_ ;
};
