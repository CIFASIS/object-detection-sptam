/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include "base_driver.hpp"

#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <dl_node/DetectionWithPoseList.h>
//#include <dl_node/DetectionList.h>
#include <sptam/StereoKFwithPose.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

#ifdef USE_LOOPCLOSURE
#include "../sptam/loopclosing/LCDetector.hpp"
#endif

namespace sptam
{

class stereo_driver : public base_driver
{
  public:

    stereo_driver(ros::NodeHandle& nh, ros::NodeHandle& nhp);

  protected:
    

    void processTrackingReport(const uint32_t seq, const ros::Time& time, const TrackingReport& report) const override;

  private:

    // We need independent feature detector and descriptor objects because
    // their corresponfing detect() and compute() options are not thread safe.
    cv::Ptr<cv::FeatureDetector> feature_detector_left_, feature_detector_right_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_left_, descriptor_extractor_right_;

    double frustum_near_plane_distance_, frustum_far_plane_distance_;

    std::unique_ptr<CameraParameters> cameraParametersLeft_;
    std::unique_ptr<CameraParameters> cameraParametersRight_;

    double stereo_baseline_;
    cv::Rect left_roi_, right_roi_;

  // ROS variables
    
    ros::Publisher pub_kf_img_l_, pub_kf_img_r_, pub_kf_info_l_, pub_kf_info_r_;
    ros::Publisher mapDetPub_ ;

    size_t lastImageSeq_,keyframeCount_;

    image_transport::ImageTransport imgTransport_;

    // Theese are for debugging purposes when the
    // SHOW_TRACKED_FRAMES compiler flag is enabled.
    image_transport::Publisher stereoFrame_Pub_, stereoFrameAfter_Pub_,
                               leftFrame_Pub_, rightFrame_Pub_,
                               leftFrameAfter_Pub_, rightFrameAfter_Pub_;

    // Subscribers for the input topics
    message_filters::Subscriber<sensor_msgs::Image> sub_img_l_, sub_img_r_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_l_, sub_info_r_;

    // syncronizer for image messages. We don't know a priory which one
    // will be used, so we have to define both, no common interface :/

    // Exact time image topic synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    // Approximate time image topic synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    /* 
    // Subscriber for detection
    message_filters::Subscriber<dl_node::DetectionList> sub_detect_l_ ;
    */
    ros::Subscriber sub_detect_l_ ;


  // helper functions

    /**
     * @brief
     *   Detection callback.
     *   
     */

    // DetectionList DetectionWithPoseList
    void onDetection(const dl_node::DetectionWithPoseListConstPtr& detections_left) ;



    /**
     * @brief
     *   Syncronized image callback. Tracks the features on the new images
     *   to compute the current robot position.
     */
    void onImages(
      const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& info_msg_l,
      const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& info_msg_r
    );

    /**
     * @brief compute stereo baseline, ROI's and FOV's from camera calibration messages.
     */
    void loadCameraCalibration( const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                const sensor_msgs::CameraInfoConstPtr& r_info_msg );
}; // stereo_driver

}; // sptam
