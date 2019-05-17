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
      ObjectMeasurement(const size_t obj_class, const double_t score, const sptam::Map::SharedKeyFrame& src_kf, const CameraPose& src_kf_delta, const size_t x1, const size_t y1, const size_t x2, const size_t y2) ; 

      inline const size_t GetClass() const
      { return obj_class_; }
      
      inline const double_t GetScore() const
      { return score_; }

      /*
      Get pose from which object was detected. src_kf_delta_ is the correction that must be applied to the keyfram
      It takes you from the corrected frame to the keyframe frame
      */
      inline const Eigen::Quaterniond GetOrientation() const
      { return src_kf_->GetOrientation() * src_kf_delta_.GetOrientationQuaternion() ; }

      inline const Eigen::Vector3d GetPosition() const
      { return src_kf_->GetCameraPose().GetOrientationMatrix() * src_kf_delta_.GetPosition() + src_kf_->GetPosition() ;}

      
      inline CameraPose GetCameraPose() const
      { return CameraPose(this->GetPosition(),this->GetOrientation(),src_kf_->GetCameraPose().covariance()) ; }
      
      inline Eigen::Vector3d ToWorld( const Eigen::Vector3d& x ) const
      { return this->GetOrientation().toRotationMatrix() * x + this->GetPosition() ; } 

      inline Eigen::Vector3d ToWorld(const double_t u, const double_t v)
      {
         double_t depth_ = this->depth() ;
         return this->ToWorld(Eigen::Vector3d((u - imagewidth_/2.0)  * (depth_/focal_),
                                              (v - imageheight_/2.0) * (depth_/focal_),
                                               depth_)) ;
      }

      cv::Matx44d computeTransformation_cv()
      {
         // R = O'
         const Eigen::Matrix3d rotationMatrix = this->GetOrientation().toRotationMatrix().transpose();

         // t = -R * C where C is the camera position
         const Eigen::Vector3d translation = -rotationMatrix * this->GetPosition()  ;

         return cv::Matx44d(rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2), translation[0],
                            rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2), translation[1],
                            rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2), translation[2],
                            0, 0, 0, 1);
      }
 

      inline Eigen::Vector3d WorldToCamera(const Eigen::Vector3d point) {
         return cv2eigen(transform(this->computeTransformation_cv(), eigen2cv( point ))) ;
      }



      /*WILL BE DELETED*/
      const Camera& GetSrcCamera() const
      { 
        //const CameraPose updated_pose = GetCameraPose() ;
        //src_kf_cam_.UpdatePose(updated_pose) ;
        return  Camera(this->GetCameraPose(),src_kf_->GetFrameLeft().GetCamera().GetCalibration()) ;
      }
      //
      /*
 *    Associate object detection with a map point to help set object depth 
      */
      inline void SetDepth(const sptam::Map::SharedPoint& depthPoint) 
      {
        depthPoint_ = depthPoint ;
      }

      inline const sptam::Map::SharedPoint GetDepthPoint() const
      {
        return depthPoint_ ;
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
         return this->ToWorld(bbox_center_CF) ;
      }

      inline double_t Area() {
        double_t depth_ = this->depth() ;
        return (x2_ - x1_) * (y2_ - y1_) * (depth_/focal_) * (depth_/focal_) ;
      }

                 
                         
                          
      Eigen::Vector3d GetNormal()
      {
            Eigen::Vector3d bbox_center_WF = this->GetBBoxCenter_World() ;
            Eigen::Vector3d normal = bbox_center_WF - this->GetPosition() ;
            normal.normalize() ;                             
            return normal ; 
      }
      
     
      inline PointPair GetBBoxCorners_World() 
      { 
         double_t depth_ = this->depth() ;
         // Translado a coordenadas respecto a punto principal y "expando" las coordenadas proporcional a la profundidad
         return PointPair( this->ToWorld(Eigen::Vector3d((x1_-imagewidth_/2.0)*(depth_/focal_), (y1_-imageheight_/2.0)*(depth_/focal_), depth_)),this->ToWorld(Eigen::Vector3d((x2_-imagewidth_/2.0)*(depth_/focal_), (y2_-imageheight_/2.0)*(depth_/focal_), depth_))) ;
      }
 


             
      Camera GetBBoxCamera(const Eigen::Vector3d& up_vector, const CameraParameters& cp_in); 
      
      // up_vector es respecto al frame de la camara
      Eigen::Matrix3d GetObjRotationFromCamera(const double_t angle, Eigen::Vector3d& up_vector) ;
      
      EigenPair GetObjTranslationFromCamera(const Eigen::Matrix3d& rotation,const Eigen::Vector3d& objx,const Eigen::Vector3d& objX,
                                                  const Eigen::Vector3d& objy, const Eigen::Vector3d& objY) ;
      

      Eigen::Vector3d GetObjPose(const double_t angle, Eigen::Vector3d& up, const Eigen::Vector3d& D, Eigen::Matrix3d& Rotation, EigenPair& EP) ; 
   
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
     
     //Be careful with this function. Is it what we want?
     inline double_t depth() {
       if (depthPoint_ != NULL) 
         return (this->GetPosition() - depthPoint_->GetPosition()).norm() ;
       else
         return focal_ ;
     }

     sptam::Map::SharedKeyFrame src_kf_ ;
     CameraPose src_kf_delta_ ;
     size_t obj_class_,x1_,y1_,x2_,y2_ ;
     double_t score_ ;
     //Camera src_cam_ ;
     sptam::Map::SharedPoint depthPoint_ ;
     double_t imagewidth_ ;
     double_t imageheight_ ;
     float focal_ ;
     Eigen::Vector3d dim_ ;
    // Camera src_kf_cam_ ;
     //PointPair bbPoints_ ; 

};





