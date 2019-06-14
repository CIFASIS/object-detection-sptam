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
#include "../../Localization/Tracker.h"
#include "../../Gui/PointCloud.h"

int main()
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

  // Compute Camera derivatives
  Matd cameraDerivatives;
  ComputeCameraDerivatives(intrinsic, cameraDerivatives);

  // Initial CameraPose (rotation and translation)

  Matd rotation = cv::Mat::eye(3,3,CV_64FC1);
  Matd translation = (Matd(3,1) << 0, 0, 0);

  CameraPose initCameraPose;
  initCameraPose.SetRotation(rotation);
  initCameraPose.SetTranslation(translation);

  Matd projectionMatrix;
  ComputeProjectionGivenCamPose(intrinsic, rotation, translation, projectionMatrix);

  std::cout << "projection Matrix: " << projectionMatrix << std::endl;

  cv::vector<CameraPose> predictedCameraPoses;
  cv::vector<Matd> predictedProjections;


// Predicted CameraPose 1
  CameraPose predictedCameraPose1;

  Matd predictedRotation1 = (Matd(3,3) << 1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1);

//  Matd predictedTranslation1 = (Matd(3,1) << 3, 0, 0);
  Matd predictedTranslation1 = (Matd(3,1) << 0, 0, 0);

  predictedCameraPose1.SetRotation(predictedRotation1);
  predictedCameraPose1.SetTranslation(predictedTranslation1);

  Matd predictedProjection1;
  ComputeProjectionGivenCamPose(intrinsic, predictedRotation1, predictedTranslation1, predictedProjection1);

  std::cout << "Prediction Rotation: " << predictedCameraPose1.GetRotation() << std::endl;
  std::cout << "Prediction Position: " << Matd(predictedCameraPose1.GetPosition()) << std::endl;

  predictedCameraPoses.push_back(predictedCameraPose1);
  predictedProjections.push_back(predictedProjection1);


// Predicted CameraPose 2
  CameraPose predictedCameraPose2;
  Matd predictedRotation2 = (Matd(3,3) << 0.9397, 0, 0.3420,
                                          0, 1, 0,
                                         -0.3420, 0, 0.9397);

//  Matd predictedRotation2 = (Matd(3,3) << 1, 0, 0,
//                                          0, 1, 0,
//                                          0, 0, 1);


//  Matd predictedTranslation2 = (Matd(3,1) << 5, 0, 0);
  Matd predictedTranslation2 = (Matd(3,1) << 0, 0, 0);

  predictedCameraPose2.SetRotation(predictedRotation2);
  predictedCameraPose2.SetTranslation(predictedTranslation2);

  Matd predictedProjection2;
  ComputeProjection(intrinsic, predictedRotation2, predictedTranslation2, predictedProjection2);

  predictedCameraPoses.push_back(predictedCameraPose2);
  predictedProjections.push_back(predictedProjection2);

  // Predicted CameraPose 3
  CameraPose predictedCameraPose3;

//  Matd predictedRotation3 = (Matd(3,3) << 1, 0, 0,
//                                          0, 1, 0,
//                                          0, 0, 1);

  Matd predictedRotation3 = (Matd(3,3) << 0.8660, 0, 0.5000,
                                         0, 1, 0,
                                        -0.5000, 0, 0.8660);


//  Matd predictedTranslation3 = (Matd(3,1) << 7, 0, 0);
  Matd predictedTranslation3 = (Matd(3,1) << 0, 0, 0);

  predictedCameraPose3.SetRotation(predictedRotation3);
  predictedCameraPose3.SetTranslation(predictedTranslation3);

  Matd predictedProjection3;
  ComputeProjection(intrinsic, predictedRotation3, predictedTranslation3, predictedProjection3);

  predictedCameraPoses.push_back(predictedCameraPose3);
  predictedProjections.push_back(predictedProjection3);


  // Ground Truth cameraPose 1
  cv::vector<CameraPose> groundTruthCameraPoses;
  cv::vector<Matd> groundTruthProjections;

  CameraPose groundTruthCameraPose1;
  Matd groundTruthRotation1 = (Matd(3,3) << 0.9848, 0, 0.1736,
                                         0, 1, 0,
                                        -0.1736, 0, 0.9848);

//    Matd groundTruthRotation1 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation1 = (Matd(3,1) << 2, 0, 0);
//  Matd groundTruthTranslation1 = (Matd(3,1) << 0, 0, 0);

  groundTruthCameraPose1.SetRotation(groundTruthRotation1);
  groundTruthCameraPose1.SetTranslation(groundTruthTranslation1);


  Matd groundTruthProjection1;
  ComputeProjection(intrinsic, groundTruthRotation1, groundTruthTranslation1, groundTruthProjection1);

  groundTruthCameraPoses.push_back(groundTruthCameraPose1);
  groundTruthProjections.push_back(groundTruthProjection1);

  // Ground Truth cameraPose 2
  CameraPose groundTruthCameraPose2;
  Matd groundTruthRotation2 = (Matd(3,3) << 0.9397, 0, 0.3420,
                                            0, 1, 0,
                                            -0.3420, 0, 0.9397);

//  Matd groundTruthRotation2 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation2 = (Matd(3,1) << 4, 0, 0);
//  Matd groundTruthTranslation2 = (Matd(3,1) << 0, 0, 0);

  groundTruthCameraPose2.SetRotation(groundTruthRotation2);
  groundTruthCameraPose2.SetTranslation(groundTruthTranslation2);


  Matd groundTruthProjection2;
  ComputeProjection(intrinsic, groundTruthRotation2, groundTruthTranslation2, groundTruthProjection2);

  groundTruthCameraPoses.push_back(groundTruthCameraPose2);
  groundTruthProjections.push_back(groundTruthProjection2);

  // Ground Truth cameraPose 3
  CameraPose groundTruthCameraPose3;
  Matd groundTruthRotation3 = (Matd(3,3) << 0.8660, 0, 0.5000,
                                         0, 1, 0,
                                        -0.5000, 0, 0.8660);

//  Matd groundTruthRotation3 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation3 = (Matd(3,1) << 6, 0, 0);
//  Matd groundTruthTranslation3 = (Matd(3,1) << 0, 0, 0);

  groundTruthCameraPose3.SetRotation(groundTruthRotation3);
  groundTruthCameraPose3.SetTranslation(groundTruthTranslation3);


  Matd groundTruthProjection3;
  ComputeProjection(intrinsic, groundTruthRotation3, groundTruthTranslation3, groundTruthProjection3);

  groundTruthCameraPoses.push_back(groundTruthCameraPose3);
  groundTruthProjections.push_back(groundTruthProjection3);

  // Map initialization
  Map map;

  // Create First Keyframe
  cv::Mat frame;
  KeyFrame *keyFrame = new KeyFrame(frame);

  // Create 3D points
  Matf point3DHomogeneous;

  cv::Point3d point1(0,0,10);
  cv::Point3d point2(-10,0,9);
  cv::Point3d point3(10,0,11);
  cv::Point3d point4(0,5,8);
  cv::Point3d point5(0,-6,12);
  cv::Point3d point6(0,0,15);
  cv::Point3d point7(-15,0,9);
  cv::Point3d point8(15,0,14);
  cv::Point3d point9(0,13,13);
  cv::Point3d point10(0,-12,12);

  cv::vector<cv::Point3d> points;
  points.push_back(point1);
  points.push_back(point2);
  points.push_back(point3);
  points.push_back(point4);
  points.push_back(point5);
  points.push_back(point6);
  points.push_back(point7);
  points.push_back(point8);
  points.push_back(point9);
  points.push_back(point10);

  cv::Mat mapPointDescriptor = cv::Mat::zeros(1, 32, CV_16UC1);

  // Create MapPoints
  for(int i = 0; i < points.size(); ++i) {

    double x = points[i].x;
    double y = points[i].y;
    double z = points[i].z;

    MapPoint *mapPoint =  new MapPoint(x,y,z, mapPointDescriptor);

    // save the map Point in the map
    map.AddMapPoint(mapPoint);

    // create mapPoint information
    Measurement  measurement(mapPoint, mapPoint->GetId(), cv::Point2d(0,0), mapPointDescriptor);

    // Save map point information in the KeyFrame
    keyFrame->AddMeasurement(measurement);
  }

  // Add Keyframe to the map
  map.AddKeyFrame(keyFrame);


  // Frustum Variables
  double horizontalFOV = 110;
  double verticalFOV = 80;
  double nearPlaneDist = 0.1;
  double farPlaneDist = 20;


  // Motion Model Variables
  CameraState previousCameraState;
  CameraState currentCameraState;
  CameraState predictedCameraState;
  CameraPose currentCameraPose = initCameraPose;
  previousCameraState.SetCameraPose(currentCameraPose);
  currentCameraState.SetCameraPose(currentCameraPose);
  MotionModel motionModel(previousCameraState, 1);

  #ifdef SHOW_POINT_CLOUD
  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  PointCloud pointCloud(viewer, currentCameraPose, map);
  pointCloud.Start();
  #endif

  // Main loop
//    CameraPose updatedCameraPose = initCameraPose;
  for(int c = 0; c < groundTruthCameraPoses.size(); ++c) {

    motionModel.SetPreviousCameraState(previousCameraState);
    motionModel.PredictCameraState(currentCameraState, predictedCameraState);
    previousCameraState = currentCameraState;
    std::cout << "Predicted Camera State: " << std::endl;
    predictedCameraState.Show(); // show the predictedCameraState
//    predictedCameraState = currentCameraState; // no motion

//    FrustumCulling frustumCulling(map, predictedCameraPoses[c], horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
    FrustumCulling frustumCulling(map, predictedCameraState.GetCameraPose(), horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
    frustumCulling.ApplyFilter();

    // Get the mapPoints who are inside the Frustum
    cv::vector<MapPoint *> mapPoints;
    mapPoints = frustumCulling.GetContainedPoints();


    // Create Matches using the Ground Thruth projection
    Measurements matches;

    for(int i = 0; i < mapPoints.size(); ++i) {
      MapPoint* mapPoint = mapPoints[i];
      cv::Point3d point3d = mapPoint->GetPosition();
      Matd point3DHomo = (Matd(4,1) << point3d.x, point3d.y, point3d.z, 1);
      Matd point2DHomo =  groundTruthProjections[c] * point3DHomo;

      // Convert to inHomogeneous imagePos
      double x = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
      double y = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);

//      std::cout << "point2D: (" << x << "," << y << ")" << std::endl;

      // Agregamos un match al resultado
      FeatureMatch newMatch;
      newMatch.mapPoint = mapPoint;
      newMatch.mapPointId = mapPoint->GetId();
      newMatch.imagePos = cv::Point2d(static_cast<double>(x), static_cast<double>(y));

      matches.push_back(newMatch);
    }

    cv::Vec6d motion;
    CameraPose updatedCameraPose = predictedCameraState.GetCameraPose();

//      CameraPose updatedCameraPose = predictedCameraPoses[c];

    // make ten gauss-newton pose update iterations.
    for(int iter = 0; iter<10; ++iter) {

      // Make one gauss-newton iteration
      GaussNewton(map, matches, intrinsic, cameraDerivatives, updatedCameraPose, motion);

      // update the camera pose with the motion computed
      Matd motionSE3Mat;
      MuToSE3Mat(motion, motionSE3Mat);
      Matd updateAux = motionSE3Mat * updatedCameraPose.GetSE3Matrix(); // the invert motion is used
      updatedCameraPose.SetRotation(updateAux(cv::Rect(0,0,3,3)));
      updatedCameraPose.SetTranslation(updateAux(cv::Rect(3,0,1,3)));
    }

  currentCameraPose = updatedCameraPose;

  // Update camera state (update linear and angular velocity)
  motionModel.UpdateCameraState(updatedCameraPose, currentCameraState);

  currentCameraState.Show();

  std::cout <<  " Estimated Camera Pose: " << std::endl << updatedCameraPose.GetSE3Matrix() << std::endl;
  std::cout <<  " Ground Truth Camera Pose: " << std::endl << groundTruthCameraPoses[c].GetSE3Matrix() << std::endl;

  #ifdef SHOW_POINT_CLOUD
  pointCloud.updated = true;
  #endif

  std::cin.ignore(10, '\n' );
  std::cout <<  "Presione ENTER: " << std::endl;
  std::cout <<  "Gracias!" << std::endl;

  }

  std::cout <<  "Termino el Testing" << std::endl;

  #ifdef SHOW_POINT_CLOUD
    pointCloud.Stop();
  #endif

  // Free Space
  cv::destroyAllWindows();

  return 0;
}




