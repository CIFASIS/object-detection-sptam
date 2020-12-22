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

#ifndef STAM_H
#define STAM_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "../../../stam/StereoProcessing/StereoProcessing.h"
#include "../../../stam/Localization/Tracker.h"
#include "../../../stam/Localization/MapMaker.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace message_filters::sync_policies;

//uncomment when the input video is already undistorted (eg. a video created with libelas datasets)
#define UNDISTORTED

namespace generateInitMap {
  class GenerateInitMap {
    public:
      GenerateInitMap(ros::NodeHandle& nodeHandle, const char* parametersFileYML);
      ~GenerateInitMap(void);

      message_filters::Subscriber<Image> sub_l_image_, sub_r_image_;
      message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;

//      ros::Subscriber<Image> sub_l_image_, sub_r_image_;
//      ros::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
      typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
      typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
      boost::shared_ptr<ApproximateSync> approximate_sync_;

      void CallBack(const ImageConstPtr& l_image_msg,
                        const CameraInfoConstPtr& l_info_msg,
                        const ImageConstPtr& r_image_msg,
                        const CameraInfoConstPtr& r_info_msg);

      ros::Publisher mapPub;
      void PublishMap();


      stam::Map map_;
      CameraPose currentCameraPose;
      RectifyMaps rectifyMaps;
      MapMaker *mapMaker_;
      Tracker *tracker;
      std::vector<cv::Mat> frame;
  };
}
#endif // STAM_H
