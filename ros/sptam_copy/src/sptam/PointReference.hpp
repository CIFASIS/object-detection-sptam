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

#include"MapPoint.hpp"

/**
 * @brief Objects of this type mimic a map point. On one hand, the data
 *   used during the tracking process is copied, so it still can be used
 *   if the associated map point is deleted during map maintainance. On
 *   the other hand, a weak pointer reference to the original map point is
 *   saved along, because we need to update the corresponding descriptors
 *   at the end. The weak pointer allows os to prior check if the point
 *   still exists.
 */
class PointReference : public IPointData
{
  public:

    PointReference( sptam::Map::Point& mapPoint )
      : data_copy_( mapPoint ), weak_ptr_( mapPoint.getWeakPtr() )
    {}

    virtual inline const Eigen::Vector3d& GetPosition() const
    { return data_copy_.GetPosition(); }

    virtual inline const Eigen::Vector3d& GetNormal() const
    { return data_copy_.GetNormal(); }

    virtual inline const cv::Mat& GetDescriptor() const
    { return data_copy_.GetDescriptor(); }

    const Eigen::Matrix3d& covariance() const
    { return data_copy_.covariance(); }

    bool isActive() const
    { return weak_ptr_.isActive(); }

    // TODO aca tiene que haber un weak ptr
    sptam::Map::Point& mapPointRef()
    { assert( weak_ptr_.isActive() ); return static_cast<sptam::Map::Point&>( *weak_ptr_ ); }

    inline void updateDescriptor(const cv::Mat& descriptor)
    { (*weak_ptr_).updateDescriptor( descriptor ); }

    inline void IncreaseMeasurementCount()
    { (*weak_ptr_).IncreaseMeasurementCount(); }


  private:

    PointData data_copy_;

    sptam::Map::Point::Ptr weak_ptr_;
};
