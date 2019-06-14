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

  // Initial CameraPose (rotation and translation)

  Matd rotation = cv::Mat::eye(3,3,CV_64FC1);
  cv::Vec3d position(0,0,0);

  CameraPose initCameraPose;
  initCameraPose.SetRotation(rotation);
  initCameraPose.SetPosition(position);

  Matd projectionMatrix;
  ComputeProjectionGivenCamPose(intrinsic, rotation, Matd(position), projectionMatrix);

//  std::cout << "projection Matrix: " << projectionMatrix << std::endl;

  cv::vector<CameraPose> predictedCameraPoses;
  cv::vector<Matd> predictedProjections;


  // Predicted CameraPose 1
  CameraPose predictedCameraPose1;
  Matd predictedRotation1 = (Matd(3,3) <<1, 0, 0,
                                         0, 1, 0,
                                         0, 0, 1);
  // -19 grados
//  Matd predictedRotation1 = (Matd(3,3) << 0.9455, 0, 0.3256,
//                                          0, 1, 0,
//                                         -0.3256, 0, 0.9455);


//  Matd predictedTranslation1 = (Matd(3,1) << -11, 0, 0);
  Matd predictedTranslation1 = (Matd(3,1) << 0, 0, 0);


  predictedCameraPose1.SetRotation(predictedRotation1);
  predictedCameraPose1.SetTranslation(predictedTranslation1);

  Matd predictedProjection1;
  ComputeProjection(intrinsic, predictedRotation1, predictedTranslation1, predictedProjection1);

//  std::cout << "Prediction Rotation: " << predictedCameraPose1.GetRotation() << std::endl;
//  std::cout << "Prediction Position: " << Matd(predictedCameraPose1.GetPosition()) << std::endl;

  predictedCameraPoses.push_back(predictedCameraPose1);
  predictedProjections.push_back(predictedProjection1);


// Predicted CameraPose 2
  CameraPose predictedCameraPose2;
  Matd predictedRotation2 = (Matd(3,3) << 0.9397, 0, 0.3420,
                                          0, 1, 0,
                                         -0.3420, 0, 0.9397);


  Matd predictedTranslation2 = (Matd(3,1) << -11, 0, 0);

  predictedCameraPose2.SetRotation(predictedRotation2);
  predictedCameraPose2.SetTranslation(predictedTranslation2);

  Matd predictedProjection2;
  ComputeProjection(intrinsic, predictedRotation2, predictedTranslation2, predictedProjection2);

  predictedCameraPoses.push_back(predictedCameraPose2);
  predictedProjections.push_back(predictedProjection2);

  // Predicted CameraPose 3
  CameraPose predictedCameraPose3;
//  Matd predictedRotation3 = (Matd(3,3) << 0.9397, 0, 0.3420,
//                                          0, 1, 0,
//                                         -0.3420, 0, 0.9397);

  Matd predictedRotation3 = (Matd(3,3) << 0.8660, 0, 0.5000,
                                         0, 1, 0,
                                        -0.5000, 0, 0.8660);


  Matd predictedTranslation3 = (Matd(3,1) << -16, 0, 0);

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
  Matd groundTruthRotation1 = (Matd(3,3) << 0.9397, 0, 0.3420,
                                            0, 1, 0,
                                            -0.3420, 0, 0.9397);

//  Matd groundTruthRotation1 = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation1 = (Matd(3,1) << -1, 0, 0);
//  Matd groundTruthTranslation1 = (Matd(3,1) << 0, 0, 0);


  Matd groundTruthProjection1;
  ComputeProjection(intrinsic, groundTruthRotation1, groundTruthTranslation1, groundTruthProjection1);

  groundTruthCameraPoses.push_back(groundTruthCameraPose1);
  groundTruthProjections.push_back(groundTruthProjection1);

  // Ground Truth cameraPose 2
  CameraPose groundTruthCameraPose2;
  Matd groundTruthRotation2 = (Matd(3,3) << 0.9397, 0, 0.3420,
                                            0, 1, 0,
                                            -0.3420, 0, 0.9397);

//  Matd groundTruthRotation = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation2 = (Matd(3,1) << -10, 0, 0);

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

//  Matd groundTruthRotation = (Matd(3,3) << 1, 0, 0,
//                                           0, 1, 0,
//                                           0, 0, 1);

  Matd groundTruthTranslation3 = (Matd(3,1) << -1.5, 0, 0);

  groundTruthCameraPose3.SetRotation(groundTruthRotation3);
  groundTruthCameraPose3.SetTranslation(groundTruthTranslation3);


  Matd groundTruthProjection3;
  ComputeProjection(intrinsic, groundTruthRotation3, groundTruthTranslation3, groundTruthProjection3);

  groundTruthCameraPoses.push_back(groundTruthCameraPose3);
  groundTruthProjections.push_back(groundTruthProjection3);

  // Map initialization
  Map map = Map();

  // Create First Keyframe
  cv::Mat frame;
  KeyFrame keyFrame(frame);

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

    MapPoint mapPoint(x,y,z, mapPointDescriptor);

    // save the map Point in the map
    MapPoint& mapPointSavedInTheMap =  map.AddMapPoint(mapPoint);

    // create mapPoint information
    MapPointInfo mapPointInfo(mapPointSavedInTheMap.id, cv::Point2d(0,0), mapPointDescriptor);

    // Save map point information in the KeyFrame
    keyFrame.AddMapPointInfo(mapPointInfo);
  }

  // Add Keyframe to the map
  map.AddKeyFrame(keyFrame);

  CameraPose currentCameraPose = initCameraPose;

  // Main loop
  //  CameraPose updatedCameraPose = initCameraPose;
  for(int c = 0; c < groundTruthCameraPoses.size(); ++c) {

    // Create Matches using the Ground Thruth projection
    VectorFeatureMatch matches;

    for(int i = 0; i < map.Size(); ++i) {
      MapPoint mapPoint = map[i];
      Matd point3DHomo = (Matd(4,1) << mapPoint.positionXYZ[0], mapPoint.positionXYZ[1], mapPoint.positionXYZ[2], 1);
      Matd point2DHomo =  groundTruthProjections[c] * point3DHomo;

      // Convert to inHomogeneous imagePos
      double x = point2DHomo.at<double>(0)/point2DHomo.at<double>(2);
      double y = point2DHomo.at<double>(1)/point2DHomo.at<double>(2);

//      std::cout << "point2D: (" << x << "," << y << ")" << std::endl;

      // Agregamos un match al resultado
      FeatureMatch *newMatch = new FeatureMatch;
      newMatch->mapPointIndex = mapPoint.id;
      newMatch->imagePos[0] = static_cast<double>(x);
      newMatch->imagePos[1] = static_cast<double>(y);

      matches.push_back(newMatch);
    }

    cv::Vec6d motion;
    CameraPose updatedCameraPose = predictedCameraPoses[c];

    // make ten gauss-newton pose update iterations.
    for(int iter = 0; iter<10; ++iter) {

      // Make one gauss-newton iteration
      GaussNewton(map, matches, intrinsic, updatedCameraPose, motion);

//      std::cout << "iter:" << iter << "  motion: " << Matd(motion) << std::endl;

      // update the camera pose with the motion computed
      Matd motionSE3Mat;
      MuToSE3Mat(motion, motionSE3Mat);
//      std::cout << "iter:" << iter << "  motion Matrix : " << motionSE3Mat << std::endl;


      Matd updateAux = updatedCameraPose.GetSE3Matrix() * motionSE3Mat;
      updatedCameraPose.SetRotation(updateAux(cv::Rect(0,0,3,3)));
      updatedCameraPose.SetTranslation(updateAux(cv::Rect(3,0,1,3)));

//      updatedCameraPose.Show();
    }

  updatedCameraPose.Show();

//  Matd motionSE3Mat;
//  cv::Vec6d mu(1,0,0,0,2*3.14,0);
//  std::cout << "Mu: " << mu << std::endl;
//  std::cout << "TooN: " << std::endl;
//  MuToSE3Mat(mu, motionSE3Mat);
//  std::cout << motionSE3Mat << std::endl;
//  std::cout << "Exponential Map: " << std::endl;
//  Matd exponentialMap;
//  ExponentialMap(mu, exponentialMap);
//  std::cout << exponentialMap << std::endl;

  std::cin.ignore(10, '\n' );
  std::cout <<  "Presione ENTER: " << std::endl;
  std::cout <<  "Gracias!" << std::endl;

  }

  std::cout <<  "Termino el Testing" << std::endl;

  // Free Space
  cv::destroyAllWindows();

  return 0;
}




