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
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
#endif

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "../../sptam/utils/cv2eigen.hpp"

//colores BGR
static const cv::Vec3d COLOR_CHARTREUSE(0,255,127);
static const cv::Vec3d COLOR_PINK(147,20,255);
static const cv::Vec3d COLOR_PURPLE(128,0,128);
static const cv::Vec3d COLOR_MAROON(0,0,128);
static const cv::Vec3d COLOR_DARKCYAN(139,139,0);
static const cv::Vec3d COLOR_GOLDENROD(32,165,218);
static const cv::Vec3d COLOR_DARKVIOLET(211,0,148);
static const cv::Vec3d COLOR_LIGHTCYAN(255,255,224);
/*
 * static const cv::Scalar COLOR_BLACK(0,0,0);
 * static const cv::Scalar COLOR_RED(0,0,255);
 * static const cv::Scalar COLOR_GREEN(0,255,0);
 * static const cv::Scalar COLOR_BLUE(255,0,0);
 * static const cv::Scalar COLOR_CYAN(255,255,0);
 * static const cv::Scalar COLOR_MAGENTA(255,0,255);
 * static const cv::Scalar COLOR_YELLOW(0,255,255);
 * static const cv::Scalar COLOR_WHITE(255,255,255);
 * static const cv::Scalar COLOR_GREY(127,127,127);
 */   


struct sortByZ {
   sortByZ( const cv::Matx44d Rt ) {this->Rt = Rt ; }
   bool operator() (const sptam::Map::SharedPoint& a,sptam::Map::SharedPoint& b) const
      { return (transform(Rt, eigen2cv( a->GetPosition() )).z < transform(Rt, eigen2cv( b->GetPosition() )).z) ; }



   cv::Matx44d Rt ;
}
;

/*Por que uso filter if not, esto devuelve false si justamente ESTA a la profundidad que quiero, así filtra el resto*/
struct filterByDepth {
   filterByDepth( const cv::Matx44d Rt, const double_t depth, const double_t epsilon) {this->Rt = Rt ; this->depth = depth; this->epsilon = epsilon ;}
   bool operator() (const sptam::Map::SharedPoint& a) const
      { return ((transform(Rt, eigen2cv( a->GetPosition() )).z < depth-epsilon)||(transform(Rt, eigen2cv( a->GetPosition() )).z > depth+epsilon)) ; }



   cv::Matx44d Rt ;
   double_t depth ;
   double_t epsilon ;
}
;



cv::Matx44d computeTransformation44(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation)
{
  // R = O'
  const Eigen::Matrix3d rotationMatrix = orientation.transpose();

  // t = -R * C where C is the camera position
  const Eigen::Vector3d translation = -rotationMatrix * position;

  return cv::Matx44d(
    rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), translation[0],
    rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), translation[1],
    rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), translation[2],
    0, 0, 0, 1
  );
}



Camera getBBoxCam(const Eigen::Vector3d& bbox_center, const Eigen::Vector3d& position,const Eigen::Vector3d up_vector, const size_t width, const size_t height,const CameraParameters& cp_in) 
{
    // nueva orientacion                    
    Eigen::Vector3d z =  bbox_center - position ;  // cameraPose_.ToWorld(bbox_center) ;
    double_t f = z.norm() ;
    z.normalize() ;
    //Eigen::Vector3d y(0.0,1.0,0.0); 
    Eigen::Vector3d y = -up_vector ;
    y.normalize() ;
    Eigen::Vector3d x = y.cross(z) ;
    y = z.cross(x) ;
    
    Eigen::Matrix3d o ;
    //Orientacion o transpuesta? probar
    //o.row(0) = x ; o.row(1) = y ; o.row(2) = z ;
    o.col(0) = x ; o.col(1) = y ; o.col(2) = z ;
    Eigen::Quaterniond q(o) ; 
    //
    
    cv::Matx33d intrin(f,   0.0,   0.0,
                       0.0,   f,   0.0,
                       0.0, 0.0,   1.0);
    
    CameraPose cameraPose(position,q,Eigen::Matrix6d::Identity()) ;
    CameraParameters cameraParameters(intrin, width, height, cp_in.frustumNearPlaneDistance(), cp_in.frustumFarPlaneDistance(), cp_in.baseline()) ;
    
    return Camera(cameraPose,cameraParameters) ;
    
    
    //float bbox_focal = bbox_center.norm() ;
    //size_t bbox_sizeX = detections_left->data[i].x2 - detections_left->data[i].x1 ;
    //size_t bbox_sizeY =  detections_left->data[i].y2 - detections_left->data[i].y1 ;
    //double horizontalFOV = 2 * atan(bbox_sizeX / (2 * bbox_focal)) * 180 / M_PI ;
    //double verticalFOV = 2 * atan(bbox_sizeY / (2 * bbox_focal)) * 180 / M_PI ;
    //FrustumCulling bboxFrustum(kf_position,q,horizontalFOV,verticalFOV,frustum_near_plane_distance_,frustum_far_plane_distance_) ;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
}


//void getMapPointFromFeature()


Eigen::Vector3d back_project(const Camera& cam,const double_t u,const double_t v,const double_t z) {

    Eigen::Vector2d principal_ = cam.GetCalibration().principalPoint() ;
    Eigen::Vector2d focal_ = cam.GetCalibration().focalLengths() ; 
    double_t x = (u - principal_(0))*(z/focal_(0)) ;   
    double_t y = (v - principal_(1))*(z/focal_(1)) ;
    
    
    
    return Eigen::Vector3d(x,y,z) ;
}

void outlier_filter (sptam::Map::SharedMapPointList& point_list) {


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  for ( const sptam::Map::SharedPoint& sMapPoint : point_list ) 
  {
    const Eigen::Vector3d& epoint = sMapPoint->GetPosition() ;
    pcl::PointXYZ pcl_point ;
    pcl_point.x = epoint.x() ; 
    pcl_point.y = epoint.y() ;
    pcl_point.z = epoint.z() ;
    cloud->points.push_back(pcl_point) ;
  }
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (5);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
 
  std::cout << "Filter" << std::endl ;
  std::cout << cloud_filtered->points.size() << std::endl ;

}



