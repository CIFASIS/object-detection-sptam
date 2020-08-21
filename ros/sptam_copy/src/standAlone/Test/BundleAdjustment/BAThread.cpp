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
#include "../../StereoProcessing/StereoProcessing.h"


#define PI 3.14159265

int main(int argc, char *argv[])
{
  if (argc != 2) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Parameters File>" << std::endl;
    std::cout << std::endl;
    std::cout << "Parameters File: Path to parameters.yml file." << std::endl;
    return -1;
  }
  // read main arguments
  const char* parametersFileYML = argv[1];

  // Read the YML file
  CameraCalibration cameraCalibrationLeft;
  CameraCalibration cameraCalibrationRight;

  RectifyMaps rectifyMaps;
  ReadCalibration(parametersFileYML, cameraCalibrationLeft, cameraCalibrationRight);

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

//
//// Predicted CameraPose 1
//  CameraPose predictedCameraPose1;
//
//  Matd predictedRotation1 = (Matd(3,3) << 1, 0, 0,
//                                          0, 1, 0,
//                                          0, 0, 1);
//
////  Matd predictedRotation1 = (Matd(3,3) << 0.996194698091746,	0,	0.087155742747658,
////                                            0,	1,	0,
////                                            -0.087155742747658,	0,	0.996194698091746);
//
//  Matd predictedTranslation1 = (Matd(3,1) << 1.1, 0, 0);
////  Matd predictedTranslation1 = (Matd(3,1) << 0, 0, 0);
//
//  predictedCameraPose1.SetRotation(predictedRotation1);
//  predictedCameraPose1.SetTranslation(predictedTranslation1);
//
//  Matd predictedProjection1;
//  ComputeProjectionGivenCamPose(intrinsic, predictedRotation1, predictedTranslation1, predictedProjection1);
//
////  std::cout << "Prediction Rotation: " << predictedCameraPose1.GetRotation() << std::endl;
////  std::cout << "Prediction Position: " << Matd(predictedCameraPose1.GetPosition()) << std::endl;
//
//  predictedCameraPoses.push_back(predictedCameraPose1);
//  predictedProjections.push_back(predictedProjection1);
//
//
//// Predicted CameraPose 2
//  CameraPose predictedCameraPose2;
//
////  Matd predictedRotation2 = (Matd(3,3) << 0.996194698091746,	0,	0.087155742747658,
////                                            0,	1,	0,
////                                            -0.087155742747658,	0,	0.996194698091746);
//
//  Matd predictedRotation2 = (Matd(3,3) << 1, 0, 0,
//                                          0, 1, 0,
//                                          0, 0, 1);
//
//
//  Matd predictedTranslation2 = (Matd(3,1) << 2.1, 0, 0);
////  Matd predictedTranslation2 = (Matd(3,1) << 0, 0, 0);
//
//  predictedCameraPose2.SetRotation(predictedRotation2);
//  predictedCameraPose2.SetTranslation(predictedTranslation2);
//
//  Matd predictedProjection2;
//  ComputeProjection(intrinsic, predictedRotation2, predictedTranslation2, predictedProjection2);
//
//  predictedCameraPoses.push_back(predictedCameraPose2);
//  predictedProjections.push_back(predictedProjection2);
////
//  // Predicted CameraPose 3
//  CameraPose predictedCameraPose3;
//
//  Matd predictedRotation3 = (Matd(3,3) << 1, 0, 0,
//                                          0, 1, 0,
//                                          0, 0, 1);
//
////  Matd predictedRotation3 = (Matd(3,3) << 0.8660, 0, 0.5000,
////                                         0, 1, 0,
////                                        -0.5000, 0, 0.8660);
//
//
//  Matd predictedTranslation3 = (Matd(3,1) << 3.1, 0, 0);
////  Matd predictedTranslation3 = (Matd(3,1) << 0, 0, 0);
//
//  predictedCameraPose3.SetRotation(predictedRotation3);
//  predictedCameraPose3.SetTranslation(predictedTranslation3);
//
//  Matd predictedProjection3;
//  ComputeProjection(intrinsic, predictedRotation3, predictedTranslation3, predictedProjection3);
//
//  predictedCameraPoses.push_back(predictedCameraPose3);
//  predictedProjections.push_back(predictedProjection3);
//
//
//  // Ground Truth cameraPose 1
//  std::vector<CameraPose> groundTruthCameraPoses;
//  std::vector<Matd> groundTruthProjections;
//
//  CameraPose groundTruthCameraPose1;
////  Matd groundTruthRotation1 = (Matd(3,3) << 0.996194698091746,	0,	0.087155742747658,
////                                            0,	1,	0,
////                                            -0.087155742747658,	0,	0.996194698091746);
//
//  Matd groundTruthRotation1 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);
//
//  Matd groundTruthTranslation1 = (Matd(3,1) << 1, 0, 0);
////  Matd groundTruthTranslation1 = (Matd(3,1) << 0, 0, 0);
//
//  groundTruthCameraPose1.SetRotation(groundTruthRotation1);
//  groundTruthCameraPose1.SetTranslation(groundTruthTranslation1);
//
//
//  Matd groundTruthProjection1;
//  ComputeProjection(intrinsic, groundTruthRotation1, groundTruthTranslation1, groundTruthProjection1);
//
//  groundTruthCameraPoses.push_back(groundTruthCameraPose1);
//  groundTruthProjections.push_back(groundTruthProjection1);
//
//  // Ground Truth cameraPose 2
//  CameraPose groundTruthCameraPose2;
////  Matd groundTruthRotation2 = (Matd(3,3) << 0.984807753012208, 0,	0.17364817766693,
////                                            0,	1,	0,
////                                            -0.17364817766693,	0,	0.984807753012208);
//
//  Matd groundTruthRotation2 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);
//
//  Matd groundTruthTranslation2 = (Matd(3,1) << 2, 0, 0);
////  Matd groundTruthTranslation2 = (Matd(3,1) << 0, 0, 0);
//
//  groundTruthCameraPose2.SetRotation(groundTruthRotation2);
//  groundTruthCameraPose2.SetTranslation(groundTruthTranslation2);
//
//
//  Matd groundTruthProjection2;
//  ComputeProjection(intrinsic, groundTruthRotation2, groundTruthTranslation2, groundTruthProjection2);
//
//  groundTruthCameraPoses.push_back(groundTruthCameraPose2);
//  groundTruthProjections.push_back(groundTruthProjection2);
//
//  // Ground Truth cameraPose 3
//  CameraPose groundTruthCameraPose3;
////  Matd groundTruthRotation3 = (Matd(3,3) << 0.8660, 0, 0.5000,
////                                         0, 1, 0,
////                                        -0.5000, 0, 0.8660);
//
//  Matd groundTruthRotation3 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);
//
//  Matd groundTruthTranslation3 = (Matd(3,1) << 3, 0, 0);
////  Matd groundTruthTranslation3 = (Matd(3,1) << 0, 0, 0);
//
//  groundTruthCameraPose3.SetRotation(groundTruthRotation3);
//  groundTruthCameraPose3.SetTranslation(groundTruthTranslation3);
//
//
//  Matd groundTruthProjection3;
//  ComputeProjection(intrinsic, groundTruthRotation3, groundTruthTranslation3, groundTruthProjection3);
//
//  groundTruthCameraPoses.push_back(groundTruthCameraPose3);
//  groundTruthProjections.push_back(groundTruthProjection3);


// Create Predicted and GroundTruth cameras
  unsigned int numCameras = 8;
  for(int i = 1; i < numCameras; ++i) {

    // Predicted CameraPose 3
    CameraPose predictedCameraPose;

    double degrees = -2*i;// -8.0 - (double)i/10.0;
    double noise = 0.1; // degrees
    double thetaNoise = (noise + degrees * PI / 180.0);
    Matd predictedRotation = (Matd(3,3) << cos(thetaNoise), 0, sin(thetaNoise),
                                           0, 1, 0,
                                           -sin(thetaNoise), 0, cos(thetaNoise));

    Matd predictedTranslation = (Matd(3,1) << i + 0.1, 0, 0);

    predictedCameraPose.SetRotation(predictedRotation);
    predictedCameraPose.SetTranslation(predictedTranslation);

    Matd predictedProjection;
    ComputeProjection(intrinsic, predictedRotation, predictedTranslation, predictedProjection);

    predictedCameraPoses.push_back(predictedCameraPose);
    predictedProjections.push_back(predictedProjection);

    // Ground Truth cameraPose 3
    CameraPose groundTruthCameraPose;
    double theta = degrees * PI / 180.0;
    Matd groundTruthRotation = (Matd(3,3) << cos(theta), 0, sin(theta),
                                           0, 1, 0,
                                           -sin(theta), 0, cos(theta));

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
  KeyFrame *initKeyFrame = new KeyFrame(frame);
  initKeyFrame->bFixed = true;

  // Create 3D points
  std::vector<cv::Point3d> points;

  unsigned int NumberOfPoints = 100;
  double x, y ,z =0;
  for(unsigned int i = 0; i < NumberOfPoints; ++i) {
    x = -5 + (rand() % 10);
    y = -5 + (rand() % 10);
    z =  5;
//    x = -5 + (rand() % 10);
//    y = -5 + (rand() % 10);
//    z =  5 + (rand() % 10);
    cv::Point3d point(x,y,z);
    points.push_back(point);
  }


// add outlier
//  cv::Point3d outlier(-4,-5,5);
//  points.push_back(outlier);


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

    MapPoint *mapPoint = new MapPoint(x,y,z, mapPointDescriptor);

    // save the map Point in the map
    map.AddMapPoint(mapPoint);

    // Project MapPoint with the initial projection
    Matd point3DHomo = (Matd(4,1) << x,y,z,1);
    Matd point2DHomo = initProjection * point3DHomo;

    // Convert to inHomogeneous imagePos
    double px = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
    double py = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);
    cv::Point2d point2d(px, py);

    // create measurement
    Measurement measurement(mapPoint, mapPoint->GetId(), point2d, mapPointDescriptor);

    // Save map measurement in the KeyFrame
    initKeyFrame->AddMeasurement(measurement);
  }

  // Add Keyframe to the map
  map.AddKeyFrame(initKeyFrame);

  // Frustum Variables
  double horizontalFOV = 110;
  double verticalFOV = 80;
  double nearPlaneDist = 0.1;
  double farPlaneDist = 20;

  // Initialize Current Camera Pose With Canonical Position
  CameraPose currentCameraPose;

  #ifdef SHOW_POINT_CLOUD
  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  PointCloud pointCloud(viewer, currentCameraPose, map);
  pointCloud.Start();
  #endif

  std::vector<KeyFrame> keyFrames;

  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING, false); // utilizar NORM_HAMMING para BRIEF y ORB, utilizar NORM_L1 NORM_L2 para SURF

  //Start MapMaker thread
  MapMaker mapMaker(intrinsic, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist, descriptorMatcher, map);
  mapMaker.Start(); // start to run MapMaker thread

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
      cv::Point3d point = mapPoint->GetPosition();
      Matd point3DHomo = (Matd(4,1) << point.x, point.y, point.z, 1);
      Matd point2DHomo =  groundTruthProjections[c] * point3DHomo;

      // Convert to inHomogeneous imagePos
      double x = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
      double y = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);

      x += 5*(rand()/RAND_MAX-0.5);
      y += 5*(rand()/RAND_MAX-0.5);

      // Agregamos un match al resultado
      FeatureMatch *newMatch = new FeatureMatch;
      newMatch->mapPointId = mapPoint->GetId();
      newMatch->imagePos.x = static_cast<double>(x);
      newMatch->imagePos.y = static_cast<double>(y);

      matches.push_back(newMatch);
    }

   // save KeyFrame
    KeyFrame keyFrame = GenerateNewKeyFrame(predictedCameraPoses[c], matches, frame);
    mapMaker.AddKeyFrame(keyFrame);
  }

  std::cout << "Main waiting for MapMaker thread" << std::endl;
  sleep(1); // para asegurarse que MapMaker corra el BA

//  mapMaker.RequestReset();
  mapMaker.Stop();
  std::cout << "Stopped MapMaker Thread" << std::endl;

  currentCameraPose = map.GetCurrentKeyFrame()->GetCameraPose();

  // aviso al visualizador que cambio la nube de puntos
  pointCloud.updated = true;

  std::cout <<  "Presione ENTER: " << std::endl;
  std::cin.ignore(10, '\n' );
  std::cout <<  "Gracias!" << std::endl;

  pointCloud.Stop();

  std::cout << "Main done" << std::endl;

  return 0;
}
