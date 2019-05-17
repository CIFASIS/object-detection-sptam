#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PathBuilder
{
  public:

    PathBuilder();

  private:

    ros::NodeHandle nh_, nhp_;

    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;

    nav_msgs::Path path_msg_;

  // helper functions

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
};

PathBuilder::PathBuilder()
  : nhp_("~")
{
  pose_sub_ = nh_.subscribe("pose", 100, &PathBuilder::poseCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);
}

void PathBuilder::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
  path_msg_.header.frame_id = pose_msg->header.frame_id;

  geometry_msgs::PoseStamped aux;
  aux.header =pose_msg->header;
  aux.pose = pose_msg->pose.pose;
  path_msg_.poses.push_back( aux );

  path_pub_.publish( path_msg_ );
}

int main(int argc, char *argv[])
{
  // Override SIGINT handler
  ros::init(argc, argv, "path_builder");

  PathBuilder path_builder;

  ROS_INFO("path_builder node running...");

  ros::spin();

  return 0;
}
 
