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
 
#include "ObjectMap.hpp" 
#include "utils/macros.hpp"

#ifdef SHOW_PROFILING
#include "utils/log/Profiler.hpp"
#endif

#include <list>


namespace sptam
{
 




    ObjectMap::SharedObject ObjectMap::MaxIOUCandidate(const SharedObjectMeasurement detection,double_t& iou_val) {
        
        double_t max_iou = 0.0 ;
        // double_t min_L2 = 1000000.0 ;
        ObjectMap::SharedObject iou_match = NULL ;
        //ObjectMap::SharedObject center_match = NULL ;

        std::cout << "IOUs" << std::endl ;

        for (const auto& candidate : list_) {
            
            std::cout << "class " << candidate->GetClass() << ':' ; 
 
            if (detection->GetSrcCamera().CanView(candidate->GetPosition())) {
               
                
                const Eigen::Vector4d& cBBox = this->BBoxPrediction(detection->GetSrcCamera(),candidate) ;
                
                //Aspect Ratio
                double_t ars = detection->ARS(size_t(cBBox(0)+0.5),size_t(cBBox(1)+0.5),size_t(cBBox(2)+0.5),size_t(cBBox(3)+0.5));

                // IOU
                double_t iou = detection->IOU(size_t(cBBox(0)+0.5),size_t(cBBox(1)+0.5),size_t(cBBox(2)+0.5),size_t(cBBox(3)+0.5));
             
                if ((max_iou < iou) && (ars < 0.7)) {
                   max_iou = iou ;
                   iou_match = candidate ; 
                }

                std::cout << iou ;
                /*
                //Bbox center
                //const Eigen::Vector2d predicton_center((cBBox(0) + cBBox(2))/ 2.0, 
                //                                       (cBBox(1) + cBBox(3))/ 2.0) ;
                // double_t center_delta = (detection->GetBBoxCenter_Image() - prediction_center).norm() ;
              
               // if (min_L2 > center_delta) {
               //   min_L2 = center_delta ;
               //   center_match = candiate ; 
               //  }
               //  */
             
            }
            std::cout << std::endl ;

        }

        iou_val = max_iou ;
        return iou_match ;
   } 

   ObjectMap::SharedObject ObjectMap::MinL2Candidate(const SharedObjectMeasurement detection,double_t& l2_val) {

        double_t min_L2 = 1000000.0 ;
        ObjectMap::SharedObject center_match = NULL ;

        std::cout << "L2s" << std::endl ;
 
  
        for (const auto& candidate : list_) {
            std::cout << "class " << candidate->GetClass() << ':' ;


            if (detection->GetSrcCamera().CanView(candidate->GetPosition())) {

                const Eigen::Vector4d& cBBox = this->BBoxPrediction(detection->GetSrcCamera(),candidate) ;

                //Bbox center
                const Eigen::Vector2d pred_center((cBBox(0) + cBBox(2))/ 2.0,
                                                  (cBBox(1) + cBBox(3))/ 2.0) ;
                double_t center_delta = (detection->GetBBoxCenter_Image() - pred_center).norm() ;

                if (min_L2 > center_delta) {
                  min_L2 = center_delta ;
                  center_match = candidate ;
                }

                std::cout << center_delta ;

            }
            std::cout << std::endl ;
        }

        l2_val = min_L2 ;
        return center_match ;
   }


 



   ObjectMap::SharedObject ObjectMap::MatchDetection(const SharedObjectMeasurement detection, const sptam::Map::SharedMapPointList& points) 
   {
   
      size_t best = 0 ;
      size_t score ;
      double_t pos_error ;
      double_t min_pos_error = 10000000.0 ;
      double_t max_iou = 0.0 ;
      double_t max_class_iou = 0.0 ;
      double_t global_score = 0.0;
      ObjectMap::SharedObject bestMatch,iou_candidate,iou_class_candidate ;

      for (const auto& candidate : list_) {

         if (detection->GetSrcCamera().CanView(candidate->GetPosition())) {
           
           std::cout << "Can View" << std::endl ;
           sptam::Map::SharedMapPointMap common_points = candidate->PointIntersectionSlow(points) ;
           std::cout << "Common Points: " << common_points.size() << std::endl ;  
      
            
           const Eigen::Vector4d& cBBox = this->BBoxPrediction(detection->GetSrcCamera(),candidate) ;         
           double_t iou = detection->IOU(size_t(cBBox(0)+0.5),size_t(cBBox(1)+0.5),size_t(cBBox(2)+0.5),size_t(cBBox(3)+0.5));
    
           std::cout << iou << " <-iou " << std::endl ;

           if (iou > max_iou) {
             max_iou = iou ;
             iou_candidate = candidate ; 
           }

           if ((iou > max_class_iou) && (candidate->GetClass() == detection->GetClass())) {
             max_class_iou = iou ;
             iou_class_candidate = candidate ;
           }

        
           score = 0 ;
           for (const auto& point : common_points) score = score + point.second ;
           
           pos_error = (detection->GetBBoxCenter_World() - candidate->GetPosition()).norm() ;
           std::cout << "Position 3d Error : " << pos_error << std::endl ;

        }  

      }

      if (max_class_iou > 0.25) 
        return iou_class_candidate ;
      else if (max_iou > 0.7) 
        return iou_candidate ;
      else
        return NULL ;

   


      //if ((max_iou > 0.6) ||((max_iou > 0.3) && (iou_candidate->GetClass() == detection->GetClass()))) 
      //  return iou_candidate ;
      //else 
      //  return NULL ;

   }      


   Eigen::Vector4d ObjectMap::BBoxPrediction(const Camera& cam, const SharedObject& object) 
   {
      const cv::Matx34d& projMat = cam.GetProjection() ;    


      const Eigen::Vector3d& pos = object->GetPosition();
      const Eigen::Matrix3d& ori = object->GetOrientationMatrix() ;
      const Eigen::Vector3d& D = object->GetDimensions() ;
      Eigen::Vector3d boxPoint[] = { Eigen::Vector3d( D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                     Eigen::Vector3d(-D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                     Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                     Eigen::Vector3d( D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                     Eigen::Vector3d( D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                     Eigen::Vector3d(-D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                     Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0,-D(2)/2.0) ,
                                     Eigen::Vector3d( D(0)/2.0,-D(1)/2.0,-D(2)/2.0) } ;

      double_t x_min = 1000000.0 ;
      double_t y_min = 1000000.0 ;
      double_t x_max = -1000000.0 ;
      double_t y_max = -1000000.0 ;

      for(int i = 0 ; i < 8 ; i++) {
          Eigen::Vector3d point = ori*boxPoint[i]+pos ;
          
          cv::Point2d projection = project( projMat , eigen2cv( point ) );
          if (projection.x < x_min) {x_min = projection.x ;} 
          if (projection.x > x_max) {x_max = projection.x ;}
          if (projection.y < y_min) {y_min = projection.y ;}
          if (projection.y > y_max) {y_max = projection.y ;}

      }
        
      return Eigen::Vector4d(x_min,y_min,x_max,y_max) ; 
   }

   void ObjectMap::AddOutliers(const Camera& src_cam)
   { 
       for (const auto& object : list_) {
          if (src_cam.CanView(object->GetPosition())) {
            if (object->GetHit()) {
              object->updateHit(false) ;
              object->IncreaseInlierCount() ;
            } else {
              object->IncreaseOutlierCount() ;
            }
          }
       }

       list_.remove_if(is_deletable()) ; 

   }


/*void Map::RemoveBadPoints()
{
  #ifdef SHOW_PROFILING
    double start = GetSeg();
  #endif

  #ifdef SHOW_PROFILING
    double end = GetSeg();
    WriteToLog(" ba remove_bad_points_lock: ", start, end);
  #endif

  // CAUTION: all points are checked!!!
  std::list<sptam::Map::SharedPoint> bad_points;
  for ( sptam::Map::Point& mapPoint : graph_.mapPoints() )
    if ( mapPoint.IsBad() )
      bad_points.push_back( mapPoint );

  for ( sptam::Map::Point& mapPoint : bad_points )
    graph_.removeMapPoint( mapPoint );
}*/

} // namespace sptam
