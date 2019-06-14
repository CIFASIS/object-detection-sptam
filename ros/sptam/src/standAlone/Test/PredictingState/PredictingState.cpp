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
#include <stdio.h>
#include "../../Localization/Map.h"
#include "../../Localization/MotionModel.h"
#include "../../Matching/Matching.h"


// Uncomment to show 3D Point Cloud
#define SHOW_POINT_CLOUD

#ifdef SHOW_POINT_CLOUD
#include "../../Gui/Draw.h"
#endif

int main()
{


  CameraPose initCameraPose;
  cv::Vec3d initLinearVelocity(0,0,0);
  cv::Vec3d initAngularVelocity(0,0.1,0);

  CameraState previousCameraState(initCameraPose, initLinearVelocity, initAngularVelocity);
  CameraState currentCameraState(initCameraPose, initLinearVelocity, initAngularVelocity);
  CameraState predictedCameraState(initCameraPose, initLinearVelocity, initAngularVelocity);

  // CameraPose currentCameraPose = currentCameraState.GetCameraPose();

  MotionModel motionModel = MotionModel(previousCameraState);

  // Create First Keyframe
  cv::Mat frame;
  KeyFrame keyFrame(frame);


  // Create Map
  Map map;

  map.AddKeyFrame(keyFrame);

  // Frustum Parameters
  double horizontalFOV = 0;
  double verticalFOV = 0;
  double nearPlaneDist = 0;
  double farPlaneDist = 0;


  CameraPose currentCameraPose;

#ifdef SHOW_POINT_CLOUD
  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map);
  PointCloud pointCloud(viewer, currentCameraPose, map);
  pointCloud.SetFrustumParamenters(horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  pointCloud.Start();
#endif

  // Main loop
  while(true) {

    #ifdef SHOW_POINT_CLOUD
    // se toma el lock
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    #endif

    // Predict Next State with the motion model
    motionModel.SetPreviousCameraState(previousCameraState);
    motionModel.PredictCameraState(currentCameraState, predictedCameraState);
    previousCameraState = currentCameraState;
    currentCameraState.Show(); // show the currentCameraStateVectorFeatureMatch matches;

    // Update camera state
    CameraPose predictedCameraPoseAux = predictedCameraState.GetCameraPose();

    // Update camera state
    motionModel.UpdateCameraState(predictedCameraPoseAux, currentCameraState);

    currentCameraPose = predictedCameraPoseAux;

    std::cout <<  "Presione ENTER: " << std::endl;
    std::cin.ignore(10, '\n' );
    std::cout <<  "Gracias!" << std::endl;

    #ifdef SHOW_POINT_CLOUD
    // se libera el lock
    pointCloud.updated = true;
    updateLock.unlock();
    #endif



  }

#ifdef SHOW_POINT_CLOUD
  pointCloud.Stop();
#endif

  return 0;
}


