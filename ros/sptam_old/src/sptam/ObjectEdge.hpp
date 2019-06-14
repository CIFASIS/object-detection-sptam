#pragma once

#include <utility>      // std::pair, std::make_pair
#include <algorithm>    // std::max
#include <set>
#include <list>
#include "Map.hpp"
#include "Camera.hpp"
#include <eigen3/Eigen/Geometry>
#include "utils/cv2eigen.hpp"
#include "utils/3dgeometry.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif


class ObjectEdge ;


typedef std::shared_ptr<ObjectEdge> SharedObjectEdge ;

struct compareSharedObjectEdge {
       bool operator() (const SharedObjectEdge& p1, const SharedObjectEdge& p2) const
      { return p1.get() < p2.get(); }
};

typedef std::set< SharedObjectEdge, compareSharedObjectEdge > SharedObjectEdgeSet ;

class ObjectEdge 
{
   public:

      ObjectEdge(const sptam::Map::SharedKeyFrame& kf, const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const Eigen::Vector3d& D, const sptam::Map::SharedPoint& depthPoint, const Eigen::Vector3d& world_normal) ;
      
      inline const Eigen::Vector3d& GetTranslation() {
        double_t depth = depth_() ;
        //std::cout << depth << ' ' << this->fromWorld(depthPoint_->GetPosition())(2) << ' ' << ' '<<  (T_* (depth/T_L2_))(2)<< " " << T_(2) << std::endl ;
        return T_* (depth/T_L2_) ;
        //o tmb
        //return T_ ;
      } 
   
      inline const Eigen::Matrix3d& GetRotation() {
        return R_ ;
      }   
  
      inline const Eigen::Vector3d& GetDimensions() {
        return D_ ;
      }
   
      inline const Eigen::Vector3d& GetNormal() {
        return world_normal_ ;
      }



     inline Eigen::Vector3d GetPosition_World() {
       return kf_->GetCameraPose().GetOrientationMatrix() * this->GetTranslation() + kf_->GetCameraPose().GetPosition() ; 
       // o tmb
       // return kf_->GetCameraPose().GetOrientationMatrix() * T_ + kf_->GetCameraPose().GetPosition() ;  
     }


     inline Eigen::Matrix3d GetOrientation_World() {
        return kf_->GetCameraPose().GetOrientationMatrix() * R_ ; 
     }


   private:
      inline Eigen::Vector3d fromWorld(const Eigen::Vector3d& x) {
        Eigen::Matrix3d R = kf_->GetCameraPose().GetOrientationMatrix().transpose() ;
        return R * x - R * kf_->GetPosition() ;
      }

      inline double_t depth_() {
        return fromWorld(depthPoint_->GetPosition()).norm() ;
        //Tambien puede ser
        //return fromWorld(depthPoint_->GetPosition())(2) ;
      } 


      sptam::Map::SharedKeyFrame kf_ ;
      Eigen::Vector3d T_ ;
      double_t T_L2_ ;
      Eigen::Matrix3d R_ ;
      Eigen::Vector3d D_ ;
      Eigen::Vector3d world_normal_ ; 
      sptam::Map::SharedPoint depthPoint_ ;
};
