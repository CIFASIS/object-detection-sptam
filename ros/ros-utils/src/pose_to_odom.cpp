#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class PoseToOdom
{
  public:

    PoseToOdom();

  private:

    ros::NodeHandle nh_, nhp_;

    ros::Publisher odom_pub_;
    ros::Subscriber pose_sub_;

  // helper functions

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};

PoseToOdom::PoseToOdom()
  : nhp_("~")
{
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  pose_sub_ = nh_.subscribe("pose", 1, &PoseToOdom::poseCallback, this);
}

void PoseToOdom::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  nav_msgs::Odometry odo_msg;
  odo_msg.header = pose_msg->header;
  odo_msg.pose.pose = pose_msg->pose;

  odom_pub_.publish(odo_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pose_to_odom");

  PoseToOdom pose_to_odom;

  ros::spin();

  return 0;
}
 
