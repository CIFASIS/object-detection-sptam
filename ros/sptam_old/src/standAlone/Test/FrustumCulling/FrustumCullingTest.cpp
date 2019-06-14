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
#include "../../FrameGenerator/IFrameGenerator.h"
#include "../../FrameGenerator/CameraFrameGenerator.h"
#include "../../FrameGenerator/VideoFileFrameGenerator.h"
#include "../../Localization/MotionModel.h"
#include "../../StereoProcessing/StereoProcessing.h"
#include "../../Matching/Matching.h"
#include "../../Matching/FrustumCulling.h"

// Uncomment to show 3D Point Cloud
#define SHOW_POINT_CLOUD

#ifdef SHOW_POINT_CLOUD
#include "../../Gui/Draw.h"
#endif

//uncomment when the input video is already undistorted (eg. a video created with libelas datasets)
#define UNDISTORTED

int main(int argc, char *argv[])
{
  if (argc < 4 || argc > 5) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Parameters File> <Left Frames Source> <Right Frames Source> [Source Type]" << std::endl;
    std::cout << std::endl;
    std::cout << "Parameters File: Path to parameters.yml file." << std::endl;
    std::cout << "Left Frames Source: Path to left video format file or left video device (eg. /dev/video1)" << std::endl;
    std::cout << "Right Frame Source: Path to right video format file or right video device (eg. /dev/video0)" << std::endl;
    std::cout << "Source Type [OPTIONAL]:" << std::endl;
    std::cout << "   vid: video source type. Left frames source and right source frame are loaded from video format file." << std::endl;
    std::cout << "   cam: video source type. Left frames source and right source frame are loaded from camera device." << std::endl;    
    return -1;
  }

  const char* parametersFileYML = argv[1];
  const char* frameSourceNameLeft = argv[2];
  const char* frameSourceNameRight = argv[3];
  const char* frameSourceType = "";
  
  if(argc == 5) {
    frameSourceType = argv[4];
  }

  // Read the YML file
  CameraCalibration cameraCalibrationLeft;
  CameraCalibration cameraCalibrationRight;

  RectifyMaps rectifyMaps;
  ReadCalibration(parametersFileYML, cameraCalibrationLeft, cameraCalibrationRight);

  // Read source (video or stereo camera)
  IFrameGenerator* frameGeneratorLeft = NULL;
  IFrameGenerator* frameGeneratorRight = NULL;
  
  if (strcmp(frameSourceType, "cam") == 0) {
    frameGeneratorLeft = new CameraFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new CameraFrameGenerator(frameSourceNameRight);
  }
  else {
    frameGeneratorLeft = new VideoFileFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new VideoFileFrameGenerator(frameSourceNameRight);
  }

  if (!frameGeneratorLeft->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameLeft << std::endl;
    return -1;
  }

  if (!frameGeneratorRight->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameRight << std::endl;
    return -1;
  }  

  // Compute Rectify Projections Matrices and rectifyMaps
  cv::Size sizeLeft = cv::Size(frameGeneratorLeft->getFrameWidth(), frameGeneratorLeft->getFrameHeight());  
  cv::Size sizeRight = cv::Size(frameGeneratorRight->getFrameWidth(), frameGeneratorRight->getFrameHeight());
  Matd Q, projectionLeft, projectionRight;
  initRectifyMaps(cameraCalibrationLeft, cameraCalibrationRight, projectionLeft, projectionRight, sizeLeft, sizeRight, rectifyMaps, Q);

  // Get the Intrinsic Matrix resulting and the translation between cameras from the rectification
  Matd intrinsicLeft, rotationLeft, translationLeft;
  decomposeProjectionMatrix(projectionLeft, intrinsicLeft, rotationLeft, translationLeft);
  cameraCalibrationLeft.SetIntrinsic(intrinsicLeft);
  cameraCalibrationLeft.SetRotation(rotationLeft);
  cameraCalibrationLeft.SetTranslation(translationLeft);

  Matd intrinsicRight, rotationRight, translationRight;
  decomposeProjectionMatrix(projectionRight, intrinsicRight, rotationRight, translationRight); // devuelve matrices homogeneas (revisar)!
  cameraCalibrationRight.SetIntrinsic(intrinsicRight);
  cameraCalibrationRight.SetRotation(rotationRight);
  cameraCalibrationRight.SetTranslation(translationRight);
  
   // Set state of the camera
  State state = State(intrinsicLeft, intrinsicRight);

  // Init Motion model
  CameraState currentCameraState = CameraState();
  CameraState previousCameraState = currentCameraState;
  CameraState predictedCameraState = currentCameraState;

  MotionModel motionModel = MotionModel(currentCameraState);

  // Define de descriptor to use 
  cv::BFMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING, false);
  
  // Map initialization
  cv::vector<cv::Mat> frame(2);
  bool hasNextFrameLeft = true, hasNextFrameRight = true;
  Matd coplanarfundamentalMatrix = (Matd(3,3) << 0, 0, 0, 0, 0, 1, 0, -1, 1);
  Matd fundamentalMatrix = coplanarfundamentalMatrix;
  bool mapInitialized = false;
  Map map = Map();

  while(hasNextFrameLeft && hasNextFrameRight && !mapInitialized) {
  
    // Get Next Frames
    hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frame[0]);
    hasNextFrameRight =  frameGeneratorRight->getNextFrame(frame[1]);
  
    #ifndef UNDISTORTED
    applyMaps(frame[0], frame[1], rectifyMaps, frame[0], frame[1]);    
    #endif

    mapInitialized = MapInitialization(frame, descriptorMatcher, projectionLeft, projectionRight, fundamentalMatrix, map);
  }

  // Set Initial Map
  state.SetMap(map);
  
#ifdef SHOW_POINT_CLOUD
  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(state);
  PointCloud pointCloud(viewer, state);
  pointCloud.Start();
#endif

  // Define window to show predictions matchings
  const std::string windowPredictionsMatching = "Matching Predictions";
  cv::namedWindow(windowPredictionsMatching, 1);
  const std::string windowPredictions = "Predictions";
  cv::namedWindow(windowPredictions, 1);
 
  // Main loop
  while(hasNextFrameLeft && hasNextFrameRight) {
   
    // Get Next Frames
    // hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frame[0]);
    // hasNextFrameRight =  frameGeneratorRight->getNextFrame(frame[1]);

    hasNextFrameLeft = true;
    hasNextFrameRight =  true;

    // Predict Next State with the motion model
    motionModel.SetPreviousCameraState(previousCameraState);
    motionModel.PredictCameraState(currentCameraState, predictedCameraState, 1);
    previousCameraState = currentCameraState;
    // currentCameraState.Show();


    // Compute predicteced projection matrix
    Matd projectionLeftPredicted;
    ComputeProjectionMatrix(cameraCalibrationLeft.GetIntrinsic(), predictedCameraState.GetRotation(), Matd(predictedCameraState.GetTranslation()), projectionLeftPredicted);

    // Correction of the state predicted

    // Obtener sub-mapa de map points que son vistos desde la posicion predicha

    double horizontalFOV, verticalFOV, aspectRatio, focalLength;
    cv::Point2d principalPoint;
    double nearPlaneDist = 0.1; // focal length deberia ir aca
    double farPlaneDist = 100;

    calibrationMatrixValues(cameraCalibrationLeft.GetIntrinsic(), frame[0].size(), 0.0062496, 0.00181815, horizontalFOV, verticalFOV, focalLength, principalPoint, aspectRatio);

    std::cout << "Field of View in X: " << horizontalFOV << std::endl;
    std::cout << "Field of View in Y: " << verticalFOV << std::endl;
    std::cout << "Principal Point: " << principalPoint << std::endl;
    std::cout << "Aspect Ratio: " << aspectRatio << std::endl;

    FrustumCulling frustumCulling(map, predictedCameraState.GetCameraPose(), horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
    cv::Vec4f nearPlane, farPlane, leftPlane, rightPlane, topPlane, bottomPlane;
    frustumCulling.ApplyFilter();
    frustumCulling.GetPlanes(nearPlane, farPlane, leftPlane, rightPlane, topPlane, bottomPlane);
    
    cv::vector<MapPoint *> mapPoints;
    mapPoints = frustumCulling.GetContainedPoints();

    // Se proyectan los map points a la imagen
    cv::vector<Feature *> vectorFeaturePrediction;
    cv::vector<cv::KeyPoint> projectedKeyPoints, frameKeyPoints;
    cv::Mat projectedDescriptors, frameDescriptors;
    ProjectMapPoints(mapPoints, projectionLeftPredicted, horizontalFOV, verticalFOV, projectedKeyPoints, projectedDescriptors, vectorFeaturePrediction);

    cv::Mat imagePredictions;
    double neighborhoodSize = 13;
    frame[0].copyTo(imagePredictions);
    for(int i = 0; i < vectorFeaturePrediction.size(); ++i) {
      Feature* feature = vectorFeaturePrediction[i];
      cv::Point2d center(feature -> imagePos[0], feature -> imagePos[1]);
      double topLeftCornerX = (center.x - (neighborhoodSize / 2.0));
      double topLeftCornerY = (center.y - (neighborhoodSize / 2.0));
      cv::Rect neighborhood(topLeftCornerX, topLeftCornerY, neighborhoodSize, neighborhoodSize);
      drawPoint(imagePredictions, center, cv::Scalar(0,255,0));
      cv::rectangle(imagePredictions, neighborhood, cv::Scalar(255,0,0));
    }

    cv::imshow(windowPredictions, imagePredictions);

    // Se realiza el matching
    VectorFeatureMatch matches;
    MatchPredictedFeatures(frame[0], map, vectorFeaturePrediction, descriptorMatcher, matches);

    // Se muestra el matching
    showMatches(frame[0],
                map,
                matches,
                windowPredictionsMatching);
    
    
    std::cout << "cantidad de matches: "  << matches.size() << std::endl;

#ifdef SHOW_POINT_CLOUD
    // se toma el lock
    boost::mutex::scoped_lock updateLock(updateModelMutex);
#endif
    // DrawPlane(nearPlane,viewer,"near");
    // DrawPlane(farPlane,viewer,"far");
    // DrawPlane(leftPlane,viewer,"left");
    // DrawPlane(rightPlane,viewer,"right");
    // DrawPlane(topPlane,viewer,"top");
    // DrawPlane(bottomPlane,viewer,"bottom");
    
    currentCameraState = predictedCameraState;

    state.SetCameraPose(currentCameraState.GetCameraPose());

#ifdef SHOW_POINT_CLOUD
    // se libera el lock
    pointCloud.updated = true;
    updateLock.unlock();
#endif

    cv::waitKey(0);
  }
  
#ifdef SHOW_POINT_CLOUD
  pointCloud.Stop();
#endif 
  
  delete frameGeneratorLeft;
  delete frameGeneratorRight;
 
  cv::destroyAllWindows();
}
