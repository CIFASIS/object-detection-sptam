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

#include "Object.hpp"

#include "../ros/utils/detection.hpp"


Object::Object()
  : outlierCount_( 0 )
  , inlierCount_( 0 )
  , projectionCount_( 0 )
  , measurementCount_( 0 )
  , reliableCount_(0) 
  , hit_(true)  // is zero because, measurementCount_ is incremented always by Frame::AddMeasuement()
  , delete_(false)
  , good_(false)
{
  // TODO do we have to clone? can't we just assign it?
  ; 
}

Object::Object(const Object& obj,const size_t id)
: object_mutex_() // The copy has a diferent mutex!
, position_( obj.position_ )
, covariance_( obj.covariance_ )
, normal_( obj.normal_ )
, outlierCount_( obj.outlierCount_ )
, inlierCount_( obj.inlierCount_ )
, projectionCount_( obj.projectionCount_ )
, measurementCount_( obj.measurementCount_ )
, reliableCount_(obj.reliableCount_)
, objectPoints_(obj.objectPoints_)
, hit_(obj.hit_)
, class_(obj.class_)
, delete_(obj.delete_)
, good_(obj.good_)
{
  //mapPoint.GetDescriptor().copyTo(descriptor_);
  color_ = obj.getColor() ;
  id_ = id ;
}

const sptam::Map::SharedMapPointList Object::GetObjectMapPoints()  
{
   sptam::Map::SharedMapPointList object_points;

   for (const auto& point : objectPoints_ ) {
      object_points.push_back(point.first) ;
   }
 
   return object_points ;
    
}

const sptam::Map::SharedMapPointList Object::GetObjectDepthPoints()
{
   sptam::Map::SharedMapPointList object_points;

   for (const auto& measurement : objectMeasurements_ ) {
      sptam::Map::SharedPoint mp = measurement->GetDepthPoint() ;
      if (mp != NULL) {
        mp->updateColor(COLOR_DEPTH) ;
        object_points.push_back(mp) ;
      }
   }

   return object_points ;

}



const ObjectPointList Object::GetObjectBBoxPoints()
{
   ObjectPointList obj_bbox_points;
   
   
   for (const auto& bbox : objectMeasurements_ ) {
      ObjectMeasurement::PointPair corners = bbox->GetBBoxCorners_World() ;
      
      SharedObjectPoint corner_ul(new ObjectPoint(corners.first)) ;
      SharedObjectPoint corner_dr(new ObjectPoint(corners.second)) ;
      obj_bbox_points.push_back(corner_ul) ;
      obj_bbox_points.push_back(corner_dr) ;
      
   }

   
   return obj_bbox_points ;
}
 
 
const sptam::Map::SharedMapPointMap Object::PointIntersectionSlow(const sptam::Map::SharedMapPointList& points) {
        
       boost::shared_lock<boost::shared_mutex> lock(object_mutex_);
       
       sptam::Map::SharedMapPointMap ret ;
     
       //sptam::Map::SharedMapPointSet::iterator mapPoint_it ;
       //sptam::Map::SharedMapPointMap::iterator objPoint_it ;
      
 
       for (const auto& mapPoint : points) {
          size_t count = objectPoints_.count(mapPoint) ;
          if (count > 0) ret[mapPoint] = count  ;
       }

       //Debug
       std::cout << "Intersection" << points.size() << " " << objectPoints_.size() << " = " << ret.size() << std::endl ;

       return ret ;
    
}   
