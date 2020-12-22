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

#include <iostream>
#include "../../Gui/PointCloud.h"
#include "../../Localization/MapMaker.h"
#include "../../Matching/Matching.h"

int main(int argc, char *argv[])
{

   // Frame size
  unsigned int width = 640;
  unsigned int height = 480;

  // Intrinsic Matrix

  double focalX = 660/2;
  double focalY = 660/2;

  Matd intrinsic = (Matd(3,3) <<  focalX, 0, width/2,
                                  0, focalY, height/2,
                                  0, 0, 1 );

  // Initial CameraPose (rotation and translation)

  Matd rotation = cv::Mat::eye(3,3,CV_64FC1);
  Matd translation = (Matd(3,1) << 0, 0, 0);

  CameraPose initCameraPose;
  initCameraPose.SetRotation(rotation);
  initCameraPose.SetTranslation(translation);

  Matd initProjection;
  ComputeProjectionGivenCamPose(intrinsic, rotation, translation, initProjection);

  std::vector<CameraPose> predictedCameraPoses;
  std::vector<Matd> predictedProjections;

  std::vector<CameraPose> groundTruthCameraPoses;
  std::vector<Matd> groundTruthProjections;

  // Create Predicted and GroundTruth cameras
  unsigned int numCameras =12;
  for(unsigned int i = 1; i < numCameras; ++i) {

    // Predicted CameraPose 3
    CameraPose predictedCameraPose;

    Matd predictedRotation = (Matd(3,3) << 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1);

    Matd predictedTranslation = (Matd(3,1) << i + 0.1, 0, 0);

    predictedCameraPose.SetRotation(predictedRotation);
    predictedCameraPose.SetTranslation(predictedTranslation);

    Matd predictedProjection;
    ComputeProjection(intrinsic, predictedRotation, predictedTranslation, predictedProjection);

    predictedCameraPoses.push_back(predictedCameraPose);
    predictedProjections.push_back(predictedProjection);

    // Ground Truth cameraPose 3
    CameraPose groundTruthCameraPose;
    Matd groundTruthRotation = (Matd(3,3) << 1, 0, 0,
                                             0, 1, 0,
                                             0, 0, 1);

    Matd groundTruthTranslation = (Matd(3,1) << i, 0, 0);

    groundTruthCameraPose.SetRotation(groundTruthRotation);
    groundTruthCameraPose.SetTranslation(groundTruthTranslation);

    Matd groundTruthProjection;
    ComputeProjection(intrinsic, groundTruthRotation, groundTruthTranslation, groundTruthProjection);

    groundTruthCameraPoses.push_back(groundTruthCameraPose);
    groundTruthProjections.push_back(groundTruthProjection);

//    std::cout << "Camera Number: " << i << std::endl;
//    std::cout << "Predicted" << std::endl;
//    std::cout << predictedCameraPose << std::endl;
//    std::cout << "Ground-Truth" << std::endl;
//    std::cout << groundTruthCameraPose << std::endl;

  }


  // Map initialization
  Map map;

  // Create First Keyframe
  cv::Mat frame = cv::Mat::zeros(640,480,CV_64FC3);
  KeyFrame initKeyFrame(frame);
  initKeyFrame.bFixed = true;

  // Create 3D points
  std::vector<cv::Point3d> points;

  unsigned int NumberOfPoints = 5;
  double x, y ,z =0;
  for(unsigned int i = 0; i < NumberOfPoints; ++i) {
    x = -5 + (rand() % 10);
    y = -5 + (rand() % 10);
    z =  5 + (rand() % 10);
    cv::Point3d point(x,y,z);
    points.push_back(point);
  }


// add outlier
  cv::Point3d outlier1(-4,-5,5);
  points.push_back(outlier1);
  cv::Point3d outlier2(5,0,5);
  points.push_back(outlier2);

//  cv::Point3d point1(0,0,10);
//  cv::Point3d point2(-10,0,9);
//  cv::Point3d point3(10,0,11);
//  cv::Point3d point4(0,5,8);
//  cv::Point3d point5(0,-6,12);
//  cv::Point3d point6(0,0,15);
//  cv::Point3d point7(-15,0,9);
//  cv::Point3d point8(15,0,14);
//  cv::Point3d point9(5,3,13);
//  cv::Point3d point10(7,-4,12);

////  std::vector<cv::Point3d> points;
//  points.push_back(point1);
////  points.push_back(point2);
//  points.push_back(point3);
//  points.push_back(point4);
//  points.push_back(point5);
//  points.push_back(point6);
////  points.push_back(point7);
////  points.push_back(point8);
//  points.push_back(point9);
//  points.push_back(point10);

  cv::Mat mapPointDescriptor = cv::Mat::zeros(1, 32, CV_16UC1);

  // Create MapPoints with the initial Projection
  for(int i = 0; i < points.size(); ++i) {

    double x = points[i].x;
    double y = points[i].y;
    double z = points[i].z;

    MapPoint mapPoint(x,y,z, mapPointDescriptor);

    // save the map Point in the map
    MapPoint& mapPointSavedInTheMap =  map.AddMapPoint(mapPoint);

    // Project MapPoint with the initial projection
    Matd point3DHomo = (Matd(4,1) << x,y,z,1);
    Matd point2DHomo = initProjection * point3DHomo;

    // Convert to inHomogeneous imagePos
    double px = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
    double py = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);
    cv::Point2d point2d(px, py);

    // create mapPoint information
    Measurement measurement(mapPointSavedInTheMap.id, point2d, mapPointDescriptor);

    // Save map point information in the KeyFrame
    initKeyFrame.AddMeasurement(measurement);
  }

  //Start MapMaker thread
  MapMaker mapMaker(intrinsic, map);
  mapMaker.AddKeyFrame(initKeyFrame);

  // Frustum Variables
  double horizontalFOV = 110;
  double verticalFOV = 80;
  double nearPlaneDist = 0.1;
  double farPlaneDist = 20;

  std::vector<KeyFrame> keyFrames;

  // Main loop
  CameraPose updatedCameraPose = initCameraPose;
  for(int c = 0; c < groundTruthCameraPoses.size(); ++c) {

    FrustumCulling frustumCulling(map, predictedCameraPoses[c], horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
    frustumCulling.ApplyFilter();

    // Get the mapPoints who are inside the Frustum
    std::vector<MapPoint *> mapPoints;
    mapPoints = frustumCulling.GetContainedPoints();

    // Create Matches using the Ground Thruth projection
    Measurements matches;

    for(int i = 0; i < mapPoints.size(); ++i) {
      MapPoint* mapPoint = mapPoints[i];
      Matd point3DHomo = (Matd(4,1) << mapPoint->positionXYZ[0], mapPoint->positionXYZ[1], mapPoint->positionXYZ[2], 1);
      Matd point2DHomo =  groundTruthProjections[c] * point3DHomo;

      // Convert to inHomogeneous imagePos
      double x = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
      double y = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);

      // Agregamos un match al resultado
      FeatureMatch *newMatch = new FeatureMatch;
      newMatch->mapPointId = mapPoint->id;
      newMatch->imagePos[0] = static_cast<double>(x);
      newMatch->imagePos[1] = static_cast<double>(y);

      matches.push_back(newMatch);
    }

   // save KeyFrame
    KeyFrame keyFrame = GenerateNewKeyFrame(predictedCameraPoses[c], matches, frame);
    mapMaker.AddKeyFrame(keyFrame);
  }

  std::cout << "Main waiting for MapMaker thread" << std::endl;
  sleep(2); // para asegurarse que MapMaker corra el BA

  mapMaker.Stop();
  std::cout << "Stopped MapMaker Thread" << std::endl;

  // Initialize Current Camera Pose With Canonical Position
  CameraPose currentCameraPose;

  currentCameraPose = map.GetCurrentKeyFrame().GetCameraPose();

  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);

  FrustumCulling frustumCulling(map, currentCameraPose, horizontalFOV,  verticalFOV, nearPlaneDist, farPlaneDist);
  draw3DMapAndCurrentCameraPose(currentCameraPose, map, viewer, frustumCulling);


  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  viewer->close();

  std::cout << "Main done" << std::endl;

  return 0;
}
