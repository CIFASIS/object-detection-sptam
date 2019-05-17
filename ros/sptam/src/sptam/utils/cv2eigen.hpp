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

#include "macros.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/eigen.hpp>
#elif CV_MAJOR_VERSION == 3
//  #include <opencv2/core.hpp>
#endif


inline Eigen::Vector2d cv2eigen(const cv::Point2d& p)
{
  return Eigen::Vector2d(p.x, p.y);
}

inline Eigen::Vector3d cv2eigen(const cv::Point3d& p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Vector3d cv2eigen(const cv::Vec3d& v)
{
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

inline cv::Point3d eigen2cv(const Eigen::Vector3d& v)
{
  return cv::Point3d(v[0], v[1], v[2]);
}

inline Eigen::Quaterniond cv2eigen(const cv::Vec4d& q)
{
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

inline cv::Vec4d eigen2cv(const Eigen::Quaterniond& q)
{
return cv::Vec4d( q.w(), q.x(), q.y(), q.z() );
}

template<typename TYPE, int ROWS, int COLS>
inline cv::Matx<TYPE, ROWS, COLS> eigen2cv(const Eigen::Matrix<TYPE, ROWS, COLS>& m)
{
  cv::Matx<TYPE, ROWS, COLS> ret;
  forn(i, ROWS) forn(j, COLS)
    ret(i, j) = m(i, j);
  return ret;
}

template<typename TYPE, int ROWS, int COLS>
inline Eigen::Matrix<TYPE, ROWS, COLS> cv2eigen(const cv::Matx<TYPE, ROWS, COLS>& m)
{
  Eigen::Matrix<TYPE, ROWS, COLS> ret;
  forn(i, ROWS) forn(j, COLS)
    ret(i, j) = m(i, j);
  return ret;
}
