#include "ObjectEdge.hpp"

ObjectEdge::ObjectEdge(const sptam::Map::SharedKeyFrame& kf, const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const Eigen::Vector3d& D,const Eigen::Vector3d& world_normal) : kf_(kf),
  T_(T),
  R_(R),
  D_(D),
  world_normal_(world_normal)   
{
}



