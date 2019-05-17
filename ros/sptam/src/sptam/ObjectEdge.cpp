#include "ObjectEdge.hpp"

ObjectEdge::ObjectEdge(const sptam::Map::SharedKeyFrame& kf, const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const Eigen::Vector3d& D,const sptam::Map::SharedPoint& depthPoint,const Eigen::Vector3d& world_normal) : kf_(kf),
  T_(T),
  R_(R),
  D_(D),
  depthPoint_(depthPoint),
  world_normal_(world_normal)   
{
  T_L2_ = T_.norm() ;
}



