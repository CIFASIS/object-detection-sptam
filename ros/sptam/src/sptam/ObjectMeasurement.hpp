#pragma once

#include <utility>      // std::pair, std::make_pair
#include <algorithm>    // std::max
#include <cmath>        // std::abs
#include <set>
#include <list>
#include "Camera.hpp"
#include "Map.hpp"
#include <eigen3/Eigen/Geometry>
#include "utils/cv2eigen.hpp"
#include "utils/3dgeometry.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif


double_t depthFromMapPoints(const cv::Matx34d& projMat,const cv::Matx44d& transMat,const sptam::Map::SharedMapPointList& pointList,double_t x1x2, double_t y1y2,sptam::Map::SharedPoint& depth_point)  ;


double_t IoU(const size_t a_min_x, const size_t a_min_y, const size_t a_max_x, const size_t a_max_y,
             const size_t b_min_x, const size_t b_min_y, const size_t b_max_x, const size_t b_max_y) ;

class ObjectMeasurement ;

typedef std::shared_ptr<ObjectMeasurement> SharedObjectMeasurement;

struct compareSharedObjectMeasurent {
       bool operator() (const SharedObjectMeasurement& p1, const SharedObjectMeasurement& p2) const
      { return p1.get() < p2.get(); }
};

typedef std::set< SharedObjectMeasurement, compareSharedObjectMeasurent > SharedObjectMeasurementSet ;



class ObjectMeasurement
{
   public : 
      //typedef std::pair< sptam::Map::SharedPoint , sptam::Map::SharedPoint > PointPair ;
      typedef EigenPair PointPair ;
      typedef std::list< PointPair > PointPairList ;


   public :
       ObjectMeasurement(const size_t obj_class, const double_t score, const sptam::Map::SharedKeyFrame& src_kf, const CameraPose& src_cp_delta, const size_t x1, const size_t y1, const size_t x2, const size_t y2) ; 
  //
      inline const size_t GetClass() const
      { return obj_class_; }
      
      inline const double_t GetScore() const
      { return score_; }

      inline const CameraPose& GetSrcPose() const
      {
       return src_cp_delta_ ;
      }

      inline const Camera& GetSrcCamera() const
      { return src_cam_; }
      
      inline void SetDepth(const sptam::Map::SharedPoint& depthPoint) 
      {
        depthPoint_ = depthPoint ;
      }

      inline const sptam::Map::SharedPoint GetDepthPoint() const
      {
        return depthPoint_ ;
      }       

     
      inline PointPair GetBBoxCorners_World() 
      { 
         double_t depth_ = this->depth() ;
         // Translado a coordenadas respecto a punto principal y "expando" las coordenadas proporcional a la profundidad
         return PointPair( src_cam_.GetPose().ToWorld(Eigen::Vector3d((x1_-imagewidth_/2.0)*(depth_/focal_), (y1_-imageheight_/2.0)*(depth_/focal_), depth_)),
                           src_cam_.GetPose().ToWorld(Eigen::Vector3d((x2_-imagewidth_/2.0)*(depth_/focal_), (y2_-imageheight_/2.0)*(depth_/focal_), depth_))) ;
      }
      
      inline Eigen::Vector3d ToWorld(const double_t u, const double_t v) 
      {
         double_t depth_ = this->depth() ;
         return src_cam_.GetPose().ToWorld(Eigen::Vector3d((u - imagewidth_/2.0)*(depth_/focal_),
                                                           (v - imageheight_/2.0)*(depth_/focal_),
                                                           depth_)) ;
      }
      
      inline Eigen::Vector3d GetBBoxCenter_World() {
      
            return this->ToWorld((x1_+x2_)/2.0,(y1_+y2_)/2.0) ;
      }

      inline Eigen::Vector2d GetBBoxCenter_Image(){
            
            return  Eigen::Vector2d((x1_+x2_)/2.0,
                                    (y1_+y2_)/2.0);
  
      } 

      
      inline Eigen::Vector3d GetBBoxCenter_FocalPlane_World() {
         // bbox center in Camera Frame (0,0) in center (no offset)
         Eigen::Vector3d bbox_center_CF( (x1_+x2_)/2.0 - imagewidth_/2.0,
                                         (y1_+y2_)/2.0 - imageheight_/2.0,
                                         focal_);
         // bbox center in World Frame                                 
         return src_cam_.GetPose().ToWorld(bbox_center_CF) ;
      }

      inline double_t Area() {
        double_t depth_ = this->depth() ;
        return (x2_ - x1_) * (y2_ - y1_) * (depth_/focal_) * (depth_/focal_) ;
      }

                 
                         
                          
      Eigen::Vector3d GetNormal()
      {
            Eigen::Vector3d bbox_center_WF = this->GetBBoxCenter_World() ;
            Eigen::Vector3d normal = bbox_center_WF - src_cam_.GetPosition() ;
            normal.normalize() ;                             
            return normal ; 
      }
                   
      Camera GetBBoxCamera(const Eigen::Vector3d& up_vector, const CameraParameters& cp_in); 
      
      inline Eigen::Vector3d WorldToCamera(const Eigen::Vector3d point) {
         return cv2eigen(transform(this->computeTransformation(), eigen2cv( point ))) ;
      }
   
      // up_vector es respecto al frame de la camara
      Eigen::Matrix3d GetObjRotationFromCamera(const double_t angle, Eigen::Vector3d& up_vector) ;
      
      Eigen::Vector3d GetObjTranslationFromCamera(const Eigen::Matrix3d& rotation,const Eigen::Vector3d& objx,const Eigen::Vector3d& objX,
                                                  const Eigen::Vector3d& objy, const Eigen::Vector3d& objY) ;
      

      Eigen::Vector3d GetObjPose(const double_t angle, Eigen::Vector3d& up, const Eigen::Vector3d& D, Eigen::Matrix3d& Rotation) ; 
   
      inline double_t IOU(size_t x1,size_t y1,size_t x2,size_t y2) {
          if (x1 < 0) {x1 = 0 ;}
          if (x2 > imagewidth_) {x2 = imagewidth_ ;}
          if (y1 < 0.0) {y1 = 0 ;}
          if (y2 > imageheight_) {y2 = imageheight_ ;}
         
          if (x2 < 0) {x2 = 0 ;}
          if (x1 > imagewidth_) {x1 = imagewidth_ ;}
          if (y2 < 0.0) {y2 = 0 ;}
          if (y1 > imageheight_) {y1 = imageheight_ ;}

          return IoU(x1_,y1_,x2_,y2_,x1,y1,x2,y2) ;   
      }
      
      inline double_t ARS(size_t x1,size_t y1,size_t x2,size_t y2) {
          if (x1 < 0) {x1 = 0 ;}
          if (x2 > imagewidth_) {x2 = imagewidth_ ;}
          if (y1 < 0.0) {y1 = 0 ;}
          if (y2 > imageheight_) {y2 = imageheight_ ;}

          if (x2 < 0) {x2 = 0 ;}
          if (x1 > imagewidth_) {x1 = imagewidth_ ;}
          if (y2 < 0.0) {y2 = 0 ;}
          if (y1 > imageheight_) {y1 = imageheight_ ;}
  
          double_t ar1 = (x2 - x1) / double_t(y2-y1) ;
          double_t ar2 = (x2_ - x1_) / double_t(y2_-y1_) ;
          
          return std::abs(ar1-ar2) ;
      }



      Eigen::Vector3d ObjectNormalWorld(const double_t angle, Eigen::Vector3d& up_vector) ;

      Eigen::Vector3d GetFaceCorrection(const double_t objAngle, const Eigen::Vector3d& objDim) ;
  
 
   private :
  
     cv::Matx44d computeTransformation() ;
     
     inline double_t depth() {
       if (depthPoint_ != NULL) 
         return (src_cam_.GetPosition() - depthPoint_->GetPosition()).norm() ;
       else
         return focal_ ;
     }

     sptam::Map::SharedKeyFrame src_kf_ ;
     CameraPose src_cp_delta_ ;
     size_t obj_class_,x1_,y1_,x2_,y2_ ;
     double_t score_ ;
     Camera src_cam_ ;
     sptam::Map::SharedPoint depthPoint_ ;
     double_t imagewidth_ ;
     double_t imageheight_ ;
     float focal_ ;
     Eigen::Vector3d dim_ ;
     //PointPair bbPoints_ ;


};





