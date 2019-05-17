#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdomToPose
{
  public:

    OdomToPose();

  private:

    ros::NodeHandle nh_, nhp_;

    ros::Subscriber odom_sub_;
    ros::Publisher pose_pub_;

    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;

  // helper functions

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    std::string target_frame;
};

OdomToPose::OdomToPose()
  : nhp_("~"), listener(tf_buffer)
{
  odom_sub_ = nh_.subscribe("odom", 100, &OdomToPose::odomCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 100);
  nhp_.param("target_frame", target_frame, std::string());
}

void OdomToPose::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = odom_msg->header;
  pose_msg.pose = odom_msg->pose;

  if (pose_pub_.getNumSubscribers() > 0)
    pose_pub_.publish( pose_msg );

  geometry_msgs::TransformStamped t;
  t.transform.rotation = odom_msg->pose.pose.orientation;
  t.transform.translation.x = odom_msg->pose.pose.position.x;
  t.transform.translation.y = odom_msg->pose.pose.position.y;
  t.transform.translation.z = odom_msg->pose.pose.position.z;
  t.header = pose_msg.header;

  if (!target_frame.empty()) {
    geometry_msgs::TransformStamped target_tf = tf_buffer.lookupTransform(odom_msg->child_frame_id, target_frame, ros::Time(0), ros::Duration(1));
    tf2::Transform t1, t2;
    tf2::fromMsg(target_tf.transform, t2);
    tf2::fromMsg(t.transform, t1);
    t1 = t1 * t2;
    t.transform = tf2::toMsg(t1);
    t.child_frame_id = target_frame;
  }
  else
  {
    t.child_frame_id = odom_msg->child_frame_id;
  }

  broadcaster.sendTransform(t);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odom_to_pose");

  OdomToPose odom_to_pose;

  ROS_INFO("odom_to_pose node running...");

  ros::spin();

  return 0;
}
 
