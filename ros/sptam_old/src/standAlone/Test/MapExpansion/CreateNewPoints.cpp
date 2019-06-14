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
#include <signal.h>
#include "../../FrameGenerator/IFrameGenerator.h"
#include "../../FrameGenerator/CameraFrameGenerator.h"
#include "../../FrameGenerator/VideoFileFrameGenerator.h"
#include "../../FrameGenerator/FileSequenceFrameGenerator.h"
#include "../../FrameGenerator/ListOfFilesFrameGenerator.h"
#include "../../StereoProcessing/StereoProcessing.h"
#include "../../Localization/Tracker.h"


//uncomment when the input video is already undistorted (eg. a video created with libelas datasets)
//#define UNDISTORTED

// Variable for interruptions signals
bool programMustEnd = false; // programMustEnd is used for interrupt signals

// handler function called by interruptions signals
void interrupt_signal(int s) {
  programMustEnd = true;
}


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
    std::cout << "   dir: video source type. Left frames source and right source frame are loaded from camera device." << std::endl;
    return -1;
  }


  // read main arguments
  const char* parametersFileYML = argv[1];
  const char* frameSourceNameLeft = argv[2];
  const char* frameSourceNameRight = argv[3];
  const char* frameSourceType = "";

  if(argc == 5) {
    frameSourceType = argv[4];
  }

  // Read source (video or stereo camera)
  IFrameGenerator* frameGeneratorLeft = NULL;
  IFrameGenerator* frameGeneratorRight = NULL;

  if (strcmp(frameSourceType, "cam") == 0) {
    frameGeneratorLeft = new CameraFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new CameraFrameGenerator(frameSourceNameRight);
  }
  else if (strcmp(frameSourceType, "vid") == 0){
    frameGeneratorLeft = new VideoFileFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new VideoFileFrameGenerator(frameSourceNameRight);
  }
  else if (strcmp(frameSourceType, "dir") == 0){
    std::string filePrefixLeft = "I1_";
    std::string filePrefixRight = "I2_";
    std::string fileExtension = "png";
    int imageBeginIndex = 10; // 464;  //20550;
    int imageEndIndex =  372;// 600; //20588;
    frameGeneratorLeft = new FileSequenceFrameGenerator(frameSourceNameLeft, filePrefixLeft, fileExtension, imageBeginIndex, imageEndIndex);
    frameGeneratorRight = new FileSequenceFrameGenerator(frameSourceNameRight, filePrefixRight, fileExtension, imageBeginIndex, imageEndIndex);
  }
  else {
    frameGeneratorLeft = new ListOfFilesFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new ListOfFilesFrameGenerator(frameSourceNameRight);
  }

  if (!frameGeneratorLeft->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameLeft << std::endl;
    return -1;
  }

  if (!frameGeneratorRight->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameRight << std::endl;
    return -1;
  }

  // Read the YML file
  CameraCalibration cameraCalibrationLeft;
  CameraCalibration cameraCalibrationRight;

  RectifyMaps rectifyMaps;
  ReadCalibration(parametersFileYML, cameraCalibrationLeft, cameraCalibrationRight);

  ConfigurationManager& configurationManager = ConfigurationManager::getInstance();

  #ifndef UNDISTORTED
//  if(configurationManager.ptamParams->MustApplyUndistortionAndRectification()) {
    Matd Q, projectionLeft, projectionRight;
    initRectifyMaps(cameraCalibrationLeft, cameraCalibrationRight, projectionLeft, projectionRight, rectifyMaps, Q);

    // Get the Intrinsic Matrix resulting and the translation between cameras from the rectification
    cameraCalibrationLeft.FillFromProjection(projectionLeft);
    cameraCalibrationRight.FillFromProjection(projectionRight);
  #else
    Matd projectionLeft, projectionRight;

    ComputeProjection(cameraCalibrationLeft.GetIntrinsic(),
                      cameraCalibrationLeft.GetRotation(),
                      cameraCalibrationLeft.GetTranslation(),
                      projectionLeft);

    ComputeProjection(cameraCalibrationRight.GetIntrinsic(),
                      cameraCalibrationRight.GetRotation(),
                      cameraCalibrationRight.GetTranslation(),
                      projectionRight);
  #endif


  // Save Cameras Calibration in the configuration Manager
  configurationManager.cameraCalibrationLeft = &cameraCalibrationLeft;
  configurationManager.cameraCalibrationRight = &cameraCalibrationRight;

//  std::cout << "rotationLeft: " << cameraCalibrationLeft.GetRotation() << std::endl;
//  std::cout << "translationLeft: " << cameraCalibrationLeft.GetTranslation() << std::endl;
//  std::cout << "rotationRight: " << cameraCalibrationRight.GetRotation() << std::endl;
//  std::cout << "translationRight: " << cameraCalibrationRight.GetTranslation() << std::endl;

  // Define the descriptor to use (just for debugging)
  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING, false); // utilizar NORM_HAMMING o NORM_HAMMING2 para BRIEF, utilizar NORM_L1 NORM_L2 para SURF
//    cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_L2, false); // utilizar NORM_HAMMING o NORM_HAMMING2 para BRIEF, utilizar NORM_L1 NORM_L2 para SURF

  // Compute camera Field Of View (FOV)
  double focalLengthX = cameraCalibrationLeft.GetIntrinsic().at<double>(0);
  double focalLengthY = cameraCalibrationLeft.GetIntrinsic().at<double>(4);

  double undistortFactorSize;
  if(configurationManager.ptamParams->MustApplyUndistortionAndRectification())
    undistortFactorSize = 1.05; // when the images are undistorted their size change and we have to change also the FOV (adding 5%)
  else
    undistortFactorSize = 1;

  double horizontalFOV = 2 * atan(cameraCalibrationLeft.GetImageWidth() * undistortFactorSize / (2 * focalLengthX)) * 180 / M_PI;
  double verticalFOV = 2 * atan(cameraCalibrationLeft.GetImageHeight() * undistortFactorSize / (2* focalLengthY)) * 180 / M_PI;

  cameraCalibrationLeft.SetHorizontalFOV(horizontalFOV);
  cameraCalibrationLeft.SetVerticalFOV(verticalFOV);

  double nearPlaneDist = configurationManager.ptamParams->GetFrustumNearPlaneDist(); // focal length deberia ir aca
  double farPlaneDist = configurationManager.ptamParams->GetFrustumFarPlaneDist();


// Create the map with stereo




  // Declare variables and initialize them
  Map map = Map();
  std::vector<cv::Mat> frame(2);
  bool hasNextFrameLeft = true, hasNextFrameRight = true;

  // Initialize Current Camera Pose With Canonical Position
  CameraPose currentCameraPose;
  currentCameraPose.SetRotation(cameraCalibrationLeft.GetRotation());
  currentCameraPose.SetTranslation(cameraCalibrationLeft.GetTranslation());

  // Signal Interruptions
  signal(SIGINT, &interrupt_signal);
  signal(SIGTERM, &interrupt_signal);

  bool mapInitialized = false;

  // Main loop
  while(!mapInitialized && hasNextFrameLeft && hasNextFrameRight && !programMustEnd) {

    // Get Next Frames
    hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frame[0]);
    hasNextFrameRight =  frameGeneratorRight->getNextFrame(frame[1]);

    #ifndef UNDISTORTED
//    if(configurationManager.ptamParams->MustApplyUndistortionAndRectification()) {
      applyMaps(frame[0], frame[1], rectifyMaps, frame[0], frame[1]);
//    }
    #endif

    // Initialize Map
      mapInitialized = MapInitialization(frame, descriptorMatcher, projectionLeft, projectionRight, map);
  }

  std::cout << "Stopped MapMaker Thread" << std::endl;

  //Start visualization
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);

  FrustumCulling frustumCulling(map, currentCameraPose, horizontalFOV,  verticalFOV, nearPlaneDist, farPlaneDist);
  draw3DMapAndCurrentCameraPose(currentCameraPose, map, viewer, frustumCulling);


  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  viewer->close();

  cv::destroyAllWindows();

  // Free Space
  delete descriptorMatcher;
  delete frameGeneratorLeft;
  delete frameGeneratorRight;


  std::cout << "Main done" << std::endl;

  return 0;
}
