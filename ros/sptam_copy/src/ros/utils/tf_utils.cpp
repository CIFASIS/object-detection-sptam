#include "tf_utils.hpp"

void tf2::waitForTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     const ros::Duration& timeout,
                     tf2::Transform& sourceToTarget)
{
  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    // If this also fails, then just throw an exception.
    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
  }
}

void tf2::lookupTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     tf2::Transform& sourceToTarget)
{
  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    // If this also fails, then just throw an exception.
    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
  }
}

bool tf2::lookupTransformSafe(const tf2_ros::Buffer& tf_buffer,
                         const std::string& targetFrame,
                         const std::string& sourceFrame,
                         const ros::Time& time,
                         tf2::Transform& sourceToTarget)
{
  bool retVal = true;

  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    try
    {
      geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                    " to " << targetFrame << ". Error was " << ex.what() << "\n");

      retVal = false;
    }
  }

  // Transforming from a frame id to itself can fail when the tf tree isn't
  // being broadcast (e.g., for some bag files). This is the only failure that
  // would throw an exception, so check for this situation before giving up.
  if (not retVal)
  {
    if (targetFrame == sourceFrame)
    {
      sourceToTarget.setIdentity();
      retVal = true;
    }
  }

  return retVal;
}
