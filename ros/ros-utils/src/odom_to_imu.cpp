#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class OdomToImu
{
  public:

    OdomToImu();

  private:

    ros::NodeHandle nh_, nhp_;

    ros::Subscriber odom_sub_;
    ros::Publisher imu_pub_;

  // helper functions

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
};

OdomToImu::OdomToImu()
  : nhp_("~")
{
  odom_sub_ = nh_.subscribe("odom", 100, &OdomToImu::odomCallback, this);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 100);
}

void OdomToImu::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header = odom_msg->header;
  imu_msg.orientation = odom_msg->pose.pose.orientation;
  imu_pub_.publish( imu_msg );
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odom_to_imu");

  OdomToImu odom_to_imu;

  ROS_INFO("odom_to_imu node running...");

  ros::spin();

  return 0;
}

