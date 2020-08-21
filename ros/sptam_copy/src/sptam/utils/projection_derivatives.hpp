////////////////////////////////////////////////////////////////////////////////
// Jacobian computation functions. functions with a '2' suffix are the versions
// using my own calculations for what should be the projection derivatives.
// The other functions are the versions given and used by by g2o for BA.

#include <Eigen/Eigenvalues>

namespace Eigen
{
  typedef Matrix<double, 9, 9> Matrix9d;
}

Eigen::Matrix<double,2,6> jacobianXj(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world);

Eigen::Matrix<double,2,3> jacobianXi(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world);

//////////////////////////////////////////////////////////////////////////////// 
