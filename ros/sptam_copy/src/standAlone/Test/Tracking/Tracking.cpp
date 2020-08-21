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
#include "../../Gui/PointCloud.h"
#include <X11/Xlib.h>


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
  XInitThreads(); // sirve para que todos los threads puedan mostrar imagenes por pantalla
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


  // Read main arguments
  const char* parametersFileYML = argv[1];
  const char* frameSourceNameLeft = argv[2];
  const char* frameSourceNameRight = argv[3];
  const char* frameSourceType = "";

  if(argc == 5) {
    frameSourceType = argv[4];
  }

  // Read the YML file
  ConfigurationManager& configurationManager = ConfigurationManager::getInstance();
  bool parametersParsed =  configurationManager.loadConfigurationFromFile(parametersFileYML);

  if(!parametersParsed) {
    std::cerr << "ERROR: parameters.yml file couldn't be opened." << std::endl;
    return -1;
  }

  // Load Parameters
  CameraCalibration& cameraCalibrationLeft = *(configurationManager.cameraCalibrationLeft);
  CameraCalibration& cameraCalibrationRight = *(configurationManager.cameraCalibrationRight);
  PTAMParameters *ptamParameters = configurationManager.ptamParams;
  double nearPlaneDist = ptamParameters->GetFrustumNearPlaneDist(); // focal length deberia ir aca
  double farPlaneDist = ptamParameters->GetFrustumFarPlaneDist();
  double frameRate = ptamParameters->GetFrameRate();
  bool isSourceUndistorted = ptamParameters->IsSourceUndistorted();
  double exposure = 100;

  // Rectify and Undistorted image if its required
  RectifyMaps rectifyMaps;
  #ifndef UNDISTORTED
//  if(isSourceUndistorted) {
    Matd Q, projectionLeft, projectionRight;
    cv::Rect leftRoi, rightRoi;
    initRectifyMaps(cameraCalibrationLeft, cameraCalibrationRight, projectionLeft, projectionRight, rectifyMaps, Q, leftRoi, rightRoi);

    // Get the Intrinsic Matrix resulting and the translation between cameras from the rectification
    cameraCalibrationLeft.FillFromProjection(projectionLeft);
    cameraCalibrationRight.FillFromProjection(projectionRight);
    cameraCalibrationLeft.SetROI(leftRoi);
    cameraCalibrationRight.SetROI(rightRoi);
  #endif

//  std::cout << "projection left after reftification: " << projectionLeft << std::endl;
//  std::cout << "projection Right after reftification: " << projectionRight << std::endl;
//  std::cout << "intrinsicLeft after reftification: " << cameraCalibrationLeft.GetIntrinsic() << std::endl;
//  std::cout << "intrinsicRight after reftification: " << cameraCalibrationRight.GetIntrinsic() << std::endl;
//  std::cout << "rotationLeft: " << cameraCalibrationLeft.GetRotation() << std::endl;
//  std::cout << "translationLeft: " << cameraCalibrationLeft.GetTranslation() << std::endl;
//  std::cout << "rotationRight: " << cameraCalibrationRight.GetRotation() << std::endl;
//  std::cout << "translationRight: " << cameraCalibrationRight.GetTranslation() << std::endl;

  // Define the descriptor to use (just for debugging)
  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_HAMMING, false); // utilizar NORM_HAMMING para BRIEF y ORB, utilizar NORM_L1 NORM_L2 para SURF
//  cv::DescriptorMatcher* descriptorMatcher = new cv::BFMatcher(cv::NORM_L2, false); // utilizar NORM_HAMMING para BRIEF, utilizar NORM_L1 NORM_L2 para SURF


  // Compute camera Field Of View (FOV)
  double focalLengthX = cameraCalibrationLeft.GetIntrinsic().at<double>(0);
  double focalLengthY = cameraCalibrationLeft.GetIntrinsic().at<double>(4);

  double horizontalFOV = 2 * atan(cameraCalibrationLeft.GetImageWidth() / (2 * focalLengthX)) * 180 / M_PI;
  double verticalFOV = 2 * atan(cameraCalibrationLeft.GetImageHeight() / (2* focalLengthY)) * 180 / M_PI;

  cameraCalibrationLeft.SetHorizontalFOV(horizontalFOV);
  cameraCalibrationLeft.SetVerticalFOV(verticalFOV);


  // Read source (video or stereo camera)
  IFrameGenerator* frameGeneratorLeft = NULL;
  IFrameGenerator* frameGeneratorRight = NULL;

  if (strcmp(frameSourceType, "cam") == 0) {
    frameGeneratorLeft = new CameraFrameGenerator(frameSourceNameLeft,
                                                  cameraCalibrationLeft.GetImageWidth(),
                                                  cameraCalibrationLeft.GetImageHeight(),
                                                  15,
                                                  exposure);
    frameGeneratorRight = new CameraFrameGenerator(frameSourceNameRight,
                                                   cameraCalibrationRight.GetImageWidth(),
                                                   cameraCalibrationRight.GetImageHeight(),
                                                   15,
                                                   exposure);
  }
  else if (strcmp(frameSourceType, "vid") == 0){
    frameGeneratorLeft = new VideoFileFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new VideoFileFrameGenerator(frameSourceNameRight);
  }
  else if (strcmp(frameSourceType, "dir") == 0){
    std::string filePrefixLeft = "I1_";
    std::string filePrefixRight = "I2_";
    std::string fileExtension = "png";
    int imageBeginIndex = 449;  //20550;
    int imageEndIndex =  600; //20588;
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



  // Declare variables and initialize them
  Map map;
  MapMaker mapMaker(cameraCalibrationLeft.GetIntrinsic(), cameraCalibrationRight.GetIntrinsic(), horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist, descriptorMatcher, map);
//  mapMaker.Start(); // start to run MapMaker thread
  std::vector<cv::Mat> frame(2);
  bool hasNextFrameLeft, hasNextFrameRight;

  // Initialize Current Camera Pose With Left Canonical Position
  CameraPose currentCameraPose;
  currentCameraPose.SetRotation(cameraCalibrationLeft.GetRotation());
  currentCameraPose.SetTranslation(cameraCalibrationLeft.GetTranslation());

#ifdef SHOW_POINT_CLOUD
  //Start visualizer thread
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = create3DMapVisualizer(map, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  PointCloud pointCloud(viewer, currentCameraPose, map);
  pointCloud.SetFrustumParamenters(horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  pointCloud.Start();
#endif

  // Create Tracker instance
  Tracker tracker(map,
                  mapMaker,
                  currentCameraPose,
                  cameraCalibrationLeft.GetIntrinsic(),
                  horizontalFOV,
                  verticalFOV,
                  nearPlaneDist,
                  farPlaneDist,
                  descriptorMatcher);

  // Signal Interruptions
  signal(SIGINT, &interrupt_signal);
  signal(SIGTERM, &interrupt_signal);

  // Get First Frames for use in teh loop
  hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frame[0]);
  hasNextFrameRight =  frameGeneratorRight->getNextFrame(frame[1]);

  // Main loop
  while(hasNextFrameLeft && hasNextFrameRight && !programMustEnd) {

//    double start, end;
//    start = GetSeg();

    #ifndef UNDISTORTED
//    if(isSourceUndistorted) {
      applyMaps(frame[0], frame[1], rectifyMaps, frame[0], frame[1]);
//    }
    #endif

    tracker.TrackFrame(frame[0],frame[1]);

    #ifdef SHOW_POINT_CLOUD
    pointCloud.updated = true;
    #endif

//    cv::waitKey(0);
//
//    std::cout <<  "Presione ENTER: " << std::endl;
//    std::cin.ignore(10, '\n' );
//    std::cout <<  "Gracias!" << std::endl;

  // Get Next Frames
  hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frame[0]);
  hasNextFrameRight = frameGeneratorRight->getNextFrame(frame[1]);

//  end = GetSeg();
//  std::cout << "Tracking Time: " << (end - start) << std::endl;
  }

#ifdef SHOW_POINT_CLOUD
  // Stop visualizer Thread
  pointCloud.Stop();
#endif

  // Stop MapMaker Thread
  mapMaker.Stop();

  // Create PLY file to be read by meshlab software
  CreatePLYFile(map);

  std::cout << "Main stop" << std::endl;

  cv::destroyAllWindows();

  // Free Space
  delete descriptorMatcher;
  delete frameGeneratorLeft;
  delete frameGeneratorRight;

  return 0;
}

