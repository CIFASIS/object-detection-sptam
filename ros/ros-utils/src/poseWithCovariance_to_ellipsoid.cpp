#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <eigen3/Eigen/Eigenvalues>


namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class CovarianceEllipsoidBuilder
{
  public:

    CovarianceEllipsoidBuilder();

  private:

    ros::NodeHandle nh_, nhp_;

    ros::Subscriber pose_sub_;
    ros::Publisher covarianceEllipsoidPub_;

    visualization_msgs::Marker covarianceEllipsoidMsg_;

  // helper functions

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
};

visualization_msgs::Marker computeCovarianceEllipsoid( const geometry_msgs::PoseWithCovarianceStamped& poseWithCovarianceMsg ) {

  Eigen::Matrix6d covariance;
  for (int i=0; i < 6; ++i) {
    for (int j=0; j < 6; ++j) {
      covariance(i,j) = poseWithCovarianceMsg.pose.covariance[6*i + j];
    }
  }

  geometry_msgs::Point position = poseWithCovarianceMsg.pose.pose.position;
  ros::Time time = poseWithCovarianceMsg.header.stamp;
  std::string frame_id = poseWithCovarianceMsg.header.frame_id;

  // get the position part of the covariance matrix
  Eigen::Matrix3d positionCovariance = covariance.block(0, 0, 3, 3);

  // Compute eigenvalues and eigenvectors (they are complex numbers)
  Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver( positionCovariance );
  Eigen::VectorXcd eigValues_complex = eigenSolver.eigenvalues();
  Eigen::MatrixXcd eigVectors_complex = eigenSolver.eigenvectors();

  // Check eigenvalues and eigenvectors complex part is null
  assert( eigValues_complex.imag() == Eigen::Vector3d::Zero(3) );
  assert( eigVectors_complex.imag() == Eigen::Matrix3d::Zero(3, 3) );

  // keep real part of the complex eigenvalues and eigenvectors
  Eigen::Vector3d eigValues = eigValues_complex.real();
  Eigen::Matrix3d eigVectors = eigVectors_complex.real();

  Eigen::Quaterniond covarianceQuaternion ( eigVectors );

  visualization_msgs::Marker covarianceEllipsoid;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  covarianceEllipsoid.header.frame_id = frame_id;
  covarianceEllipsoid.header.stamp = time;

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  covarianceEllipsoid.ns = "covariance";
  covarianceEllipsoid.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  covarianceEllipsoid.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  covarianceEllipsoid.action = visualization_msgs::Marker::ADD;

  // painting the Gaussian Ellipsoid Marker
  covarianceEllipsoid.pose.position = position;
  covarianceEllipsoid.pose.orientation.x = covarianceQuaternion.x();
  covarianceEllipsoid.pose.orientation.y = covarianceQuaternion.y();
  covarianceEllipsoid.pose.orientation.z = covarianceQuaternion.z();
  covarianceEllipsoid.pose.orientation.w = covarianceQuaternion.w();
  double marker_scale = 100000; // scale factor to allow the ellipsoid be big enough
  covarianceEllipsoid.scale.x = eigValues[0] * marker_scale;
  covarianceEllipsoid.scale.y = eigValues[1] * marker_scale;
  covarianceEllipsoid.scale.z = eigValues[2] * marker_scale;

//  covarianceMarker.pose.orientation.x = 0;
//  covarianceMarker.pose.orientation.y = 0;
//  covarianceMarker.pose.orientation.z = 0;
//  covarianceMarker.pose.orientation.w = 1;
//  covarianceMarker.scale.x = positionCovariance(0,0) * marker_scale;
//  covarianceMarker.scale.y = positionCovariance(1,1) * marker_scale;
//  covarianceMarker.scale.z = positionCovariance(2,2) * marker_scale;
  covarianceEllipsoid.color.r = 1.0f;
  covarianceEllipsoid.color.g = 0.0f;
  covarianceEllipsoid.color.b = 1.0f;
  covarianceEllipsoid.color.a = 0.5;

  covarianceEllipsoid.lifetime = ros::Duration();

  return covarianceEllipsoid;
}

CovarianceEllipsoidBuilder::CovarianceEllipsoidBuilder()
  : nhp_("~")
{
  pose_sub_ = nh_.subscribe("pose", 100, &CovarianceEllipsoidBuilder::poseCallback, this);
  covarianceEllipsoidPub_ = nh_.advertise<visualization_msgs::Marker>("covariance", 100);
}

void CovarianceEllipsoidBuilder::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  covarianceEllipsoidMsg_ = computeCovarianceEllipsoid( *pose_msg );
  covarianceEllipsoidPub_.publish( covarianceEllipsoidMsg_ );
}

int main(int argc, char *argv[])
{
  // Override SIGINT handler
  ros::init(argc, argv, "path_builder");

  CovarianceEllipsoidBuilder covarianceEllipsoidBuilder;

  ROS_INFO("path_builder node running...");

  ros::spin();

  return 0;
}
 




