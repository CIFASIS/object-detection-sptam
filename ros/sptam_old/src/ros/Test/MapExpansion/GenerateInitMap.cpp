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

#include <cv_bridge/cv_bridge.h>
#include "GenerateInitMap.h"
#include <visualization_msgs/Marker.h>
#include "../../../sptam/Configuration/ConfigurationManager.h"



generateInitMap::GenerateInitMap::GenerateInitMap(ros::NodeHandle& nodeHandle, const char* parametersFileYML)
{

  // Read the YML file
  ConfigurationManager& configurationManager = ConfigurationManager::getInstance(); // Get Configuration Manager Instance
  bool parametersParsed =  configurationManager.loadConfigurationFromFile(parametersFileYML);

  if(!parametersParsed) {
    std::cerr << "ERROR: parameters.yml file couldn't be opened." << std::endl;
    // ver como matar a ros
    return;
  }

  // Load Parameters
  CameraCalibration& cameraCalibrationLeft = *(configurationManager.cameraCalibrationLeft);
  CameraCalibration& cameraCalibrationRight = *(configurationManager.cameraCalibrationRight);

  cv::FeatureDetector *featureDetector = configurationManager.featureDetector;
  cv::DescriptorExtractor *descriptorExtractor = configurationManager.descriptorExtractor;

  // Define the descriptor to use (just for debugging)
  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING, false); // utilizar NORM_HAMMING para BRIEF y ORB, utilizar NORM_L1 NORM_L2 para SURF
  //  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_L2, false); // utilizar NORM_HAMMING para BRIEF, utilizar NORM_L1 NORM_L2 para SURF

  PTAMParameters *ptamParameters = configurationManager.ptamParams;

  double nearPlaneDist = ptamParameters->GetFrustumNearPlaneDist(); // focal length deberia ir aca
  double farPlaneDist = ptamParameters->GetFrustumFarPlaneDist();

  MapMaker::STAMParameters mapper_params;
  mapper_params.matchingNeighborhood = ptamParameters->GetMatchingNeighborhood();
  mapper_params.matchingDistanceThreshold = ptamParameters->GetMatchingDistance();
  mapper_params.epipolarDistanceThreshold = ptamParameters->GetMatchingEpipolarDistance();
  mapper_params.frustumNearPlaneDist = nearPlaneDist;
  mapper_params.frustumFarPlaneDist = farPlaneDist;
  mapper_params.keyFrameDistanceThreshold = ptamParameters->GetKeyFrameDistance();

  // TODO todo esto debería estar resuelto por CameraCalibration(Configuration?)
  // Rectify and Undistorted image if its required
  #ifndef UNDISTORTED
//  if(isSourceUndistorted) {
    Matd Q, projectionLeft, projectionRight;
    cv::Rect leftRoi, rightRoi; // regiones de interes de la imagen
    initRectifyMaps(cameraCalibrationLeft,
                    cameraCalibrationRight,
                    projectionLeft,
                    projectionRight,
                    rectifyMaps,
                    Q,
                    leftRoi,
                    rightRoi);

    // Get the Intrinsic Matrix resulting and the translation between cameras from the rectification
    cameraCalibrationLeft.FillFromProjection(projectionLeft);
    cameraCalibrationRight.FillFromProjection(projectionRight);
    cameraCalibrationLeft.SetROI(leftRoi);
    cameraCalibrationRight.SetROI(rightRoi);
  #else // define Region of Interes as trivial when the image are Undistorted
  cv::Rect leftRoi(0,0,cameraCalibrationLeft.GetImageWidth(),cameraCalibrationLeft.GetImageHeight());
  cv::Rect rightRoi(0,0,cameraCalibrationRight.GetImageWidth(),cameraCalibrationRight.GetImageHeight());
  cameraCalibrationLeft.SetROI(leftRoi);
  cameraCalibrationRight.SetROI(rightRoi);
  #endif

  std::cout << "intrinsicLeft after rectification: " << cameraCalibrationLeft.GetIntrinsic() << std::endl;
  std::cout << "intrinsicRight after rectification: " << cameraCalibrationRight.GetIntrinsic() << std::endl;
  std::cout << "cameraPoseLeft: " << cameraCalibrationLeft.GetCameraPose() << std::endl;
  std::cout << "cameraPoseRight: " << cameraCalibrationRight.GetCameraPose() << std::endl;

  // save space for frames
  frame.resize(2);

  // Initialize mapMaker_
  mapMaker_ = new MapMaker( map_,
                            cameraCalibrationLeft, cameraCalibrationRight,
                            featureDetector, descriptorExtractor, descriptorMatcher,
                            mapper_params);

  // Subscribe to images messages
  sub_l_image_.subscribe(nodeHandle, "/stereo/left/image_raw", 1);
  sub_l_info_ .subscribe(nodeHandle, "/stereo/left/camera_info", 1);
  sub_r_image_.subscribe(nodeHandle, "/stereo/right/image_raw", 1);
  sub_r_info_ .subscribe(nodeHandle, "/stereo/right/camera_info", 1);

  approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(10),
                                               sub_l_image_, sub_l_info_,
                                               sub_r_image_, sub_r_info_) );
  approximate_sync_->registerCallback(boost::bind(&generateInitMap::GenerateInitMap::CallBack,
                                                  this, _1, _2, _3, _4));

  mapPub = nodeHandle.advertise<visualization_msgs::Marker>("point", 1000);
}

// esta funcion hace cualquiera por ahora
generateInitMap::GenerateInitMap::~GenerateInitMap(void){

  cv::destroyAllWindows();

  // Free Space
}

void generateInitMap::GenerateInitMap::CallBack(const ImageConstPtr& l_image_msg,
                                                const CameraInfoConstPtr& l_info_msg,
                                                const ImageConstPtr& r_image_msg,
                                                const CameraInfoConstPtr& r_info_msg)
{

  // if the map was not initialized
  if(!map_.nMapPoints()) {
    // conver image to OpenCv cv::Mat format
    cv_bridge::CvImageConstPtr bridgeLeft_ptr = cv_bridge::toCvShare(l_image_msg, "rgb8");
    cv_bridge::CvImageConstPtr bridgeRight_ptr = cv_bridge::toCvShare(r_image_msg, "rgb8");

    // save images
    frame[0] = bridgeLeft_ptr->image;
    frame[1] = bridgeRight_ptr->image;

    #ifndef UNDISTORTED
    applyMaps(rectifyMaps, frame[0], frame[1], frame[0], frame[1]);
    #endif

//    cv::imshow("Left", frame[0]);
//    cv::waitKey(30);
//    cv::imshow("Right", frame[1]);
//    cv::waitKey(30);


    // Esto no deberia ir aca, trendria que ir en el constructor
    ConfigurationManager& configurationManager = ConfigurationManager::getInstance();

    CameraCalibration cameraCalibrationLeft = *(configurationManager.cameraCalibrationLeft);
    CameraCalibration cameraCalibrationRight = *(configurationManager.cameraCalibrationRight);

    cv::Matx33d intrinsic = cameraCalibrationLeft.GetIntrinsic();

    // Initialize Map
    mapMaker_->InitFromStereo(frame[0], frame[1], intrinsic, intrinsic, cameraCalibrationLeft.GetCameraPose(), cameraCalibrationRight.GetCameraPose());

    // Publish Map To be drawn by rviz visualizer
    PublishMap();
    ROS_INFO("Map size: %d", (int) (map_.nMapPoints()));
  }
}

void generateInitMap::GenerateInitMap::PublishMap() {


  for( auto& mapPoint : map_.GetMapPoints() ) {

      // Get Point from Map
    cv::Point3d point3d = mapPoint->GetPosition();

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    // Create Ros Marker for visualization
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "pointCloud";
    marker.id = mapPoint->GetId();

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = point3d.x;
    marker.pose.position.y = point3d.y;
    marker.pose.position.z = point3d.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    mapPub.publish(marker);

//    ROS_INFO("Point published: (%f, %f, %f)", point3d.x, point3d.y,  point3d.z);
  }
}

