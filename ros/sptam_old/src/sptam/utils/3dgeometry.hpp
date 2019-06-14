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
#include <utility>      // std::pair, std::make_pair


#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/eigen.hpp>
#elif CV_MAJOR_VERSION == 3
//  #include <opencv2/core.hpp>
#endif

typedef std::pair< Eigen::Vector3d , Eigen::Vector3d > EigenPair ;

inline bool segmentIntersection(const EigenPair a, const EigenPair b, Eigen::Vector3d& ip) {
   
   double_t err = 0.01 ;
   
   Eigen::Vector3d da = a.second - a.first ;
   Eigen::Vector3d db = b.second - b.first ;
   Eigen::Vector3d dc = b.first - a.first ;
   
   Eigen::Vector3d daXdb = da.cross(db) ;
   
   double_t dc_perpendicular = dc.dot(daXdb) ;
   
   if ((dc_perpendicular > err) || (dc_perpendicular < -err))  //Lines not coplanar
     return false ;
   
   
   
   Eigen::Vector3d dcXdb = dc.cross(db) ;
   double_t s = (dcXdb.dot(daXdb)) / (daXdb.squaredNorm()) ;
   
   if ((s > -err) && (s < 1+err)) {
     
     ip = a.first + da * s ;
     return true ;
     
   } else return false ;
   
}


