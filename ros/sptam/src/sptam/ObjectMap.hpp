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

#include <vector>
#include <list>
#include <map>
#include "Object.hpp"
#include "ObjectMeasurement.hpp"
#include "MapPoint.hpp"
#include "StereoFrame.hpp"
#include "Measurement.hpp"
#include "utils/CovisibilityGraph.hpp"
#include "utils/Iterable.hpp"
#include "utils/cv2eigen.hpp"


#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>




namespace sptam
{


struct is_deletable {
  bool operator() (const std::shared_ptr<Object>& obj) { return obj->IsDeletable(); }
};

/**
 * This class represents the visibility map of the environment.
 * It is composed by 3D points in space, 6DOF camera views, and
 * the relationships between them, called measurements.
 */
class ObjectMap
{
  public:
    //class Object ;
   
    typedef std::shared_ptr<Object> SharedObject;

    typedef std::list<SharedObject> ObjectList;

    typedef ObjectList::iterator ObjectListIt ; 
    
    typedef std::map< SharedObjectMeasurement, SharedObject, compareSharedObjectMeasurent > SharedObjectMeasurementMap ;

    inline SharedObject addObject(const Object& obj) 
    {
      boost::unique_lock<boost::shared_mutex> lock(objectmap_mutex_);
     
      currentId_++ ;

      SharedObject sO(new Object( obj,currentId_ ));
      list_.push_back(sO);
      return sO ;
    }
      
    inline const ObjectList& GetObjects() 
    {
       boost::shared_lock<boost::shared_mutex> lock(objectmap_mutex_);

       return list_ ;
    }

    inline void Reset() 
    {
       boost::unique_lock<boost::shared_mutex> lock(objectmap_mutex_);

       list_.erase(list_.begin(),list_.end()) ; 
    }

    inline void Print() {
      int i = 1 ;
      for (const auto& object : list_) {
          std::cout << "Object " << i++ << " cls :" << object->GetClass() << std::endl;
      }
    } 

    /*
    inline void Remove(const SharedObject obj) {
      boost::unique_lock<boost::shared_mutex> lock(objectmap_mutex_);
 
      //deleted_.push_back(obj) ;
      std::cout << "B" << std::endl ;
      list_.remove(obj) ;
      std::cout << "C" << std::endl ;
    }   
    */
 
    const ObjectPointList GetObjectPoints()
    {
       ObjectPointList ret ;
       for (const auto& object : list_) {
           if (!(object->IsBad())) {
             ObjectPointList b =  object->GetObjectBBoxPoints() ;
             //ObjectPointList b = object->GetObjectBBoxPoints2() ;
             ret.splice(ret.end(), b, b.begin(),b.end()) ; //concat?
           }
       }
       return ret ;
    }
    
    SharedObject MaxIOUCandidate(const SharedObjectMeasurement detection, double_t& iou_val) ;

    SharedObject MinL2Candidate(const SharedObjectMeasurement detection, double_t& l2_val) ;

   
 
    SharedObject MatchDetection(const SharedObjectMeasurement detection, const sptam::Map::SharedMapPointList& points) ;


    void AddOutliers(const Camera& src_cam) ;

   Eigen::Vector4d BBoxPrediction(const Camera& cam, const SharedObject& object) ;

/*
    inline SharedObject AddObjectMeasurement(const sptam::Map::SharedMapPointList& points)
    {     

       double_t max_ratio = 0.0 ;
       size_t max_intersection = 0 ;
       SharedObject candidate ;

       std::cout << "ObjectMap size: " << list_.size() << std::endl ;

       for (const auto& object : list_) {
         size_t intersection = object->PointIntersectionSlow(points) ;
         if (intersection > 0) {
            double_t ratio = double_t(intersection) / object->GetPointSize() ;
            if (ratio > max_ratio) {
               candidate = object ;
               max_ratio = ratio ;
               max_intersection = intersection ;
            }
         }
       }

       if ((max_ratio > POINT_RATIO) || (max_intersection > POINT_THRESH)) {
         return candidate ;
       } else 
         return NULL ;
    } 
*/
    
  private:
    ObjectList list_ ;
    ObjectList deleted_ ;
    SharedObjectMeasurementMap measurementMap_ ;
   
    mutable boost::shared_mutex objectmap_mutex_;

    size_t currentId_ = 0 ;
 
    const double_t POS_THRESH = 1.1 ; 
    const double_t POINT_RATIO=0.15 ;
    const size_t POINT_THRESH= 4 ;
  
};


} // namespace sptam
