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
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
  if (argc < 4 || argc > 5) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Parameters File> <Left Frames Source> <Right Frames Source> [Source Type]" << std::endl;
    std::cout << std::endl;
    std::cout << "Parameters File: Path to parameters.xml file." << std::endl;
    std::cout << "Left Frames Source: Path to left video format file or left video device (eg. /dev/video1)" << std::endl;
    std::cout << "Right Frame Source: Path to right video format file or right video device (eg. /dev/video0)" << std::endl;
    std::cout << "Source Type [OPTIONAL]:" << std::endl;
    std::cout << "   vid: video source type. Left frames source and right source frame are loaded from video format file." << std::endl;
    std::cout << "   cam: video source type. Left frames source and right source frame are loaded from camera device." << std::endl;    
    return -1;
  }

  const char* parametersFileXml = argv[1];
  const char* frameSourceNameLeft = argv[2];
  const char* frameSourceNameRight = argv[3];
  const char* frameSourceType = "";
  
  if(argc == 5) {
    frameSourceType = argv[4];
  } 
 
  
  IFrameGenerator* frameGeneratorLeft = NULL;
  IFrameGenerator* frameGeneratorRight = NULL;
  
  if (strcmp(frameSourceType, "cam") == 0) {
    frameGeneratorLeft = new CameraFrameGenerator(frameSourceNameLeft);
    frameGeneratorRight = new CameraFrameGenerator(frameSourceNameRight);
  }

  if (!frameGeneratorLeft->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameLeft << std::endl;
    return -1;
  }

  if (!frameGeneratorRight->init()){
    std::cerr << "No se pudo iniciar el generador de frames a partir de " << frameSourceNameRight << std::endl;
    return -1;
  }  

  const std::string windowFrameLeft = "Left Camera";
  cv::namedWindow(windowFrameLeft, 1);
  const std::string windowFrameRight = "Right Camera";
  cv::namedWindow(windowFrameRight, 1);

  cv::Mat frameLeft, frameRight;

  bool hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frameLeft);
  bool hasNextFrameRight =  frameGeneratorRight->getNextFrame(frameRight);

  while(hasNextFrameLeft && hasNextFrameRight) {

    cv::imshow(windowFrameLeft, frameLeft);
    cv::imshow(windowFrameRight, frameRight);
    
    cv::waitKey(1);
    
    hasNextFrameLeft = frameGeneratorLeft->getNextFrame(frameLeft);
    hasNextFrameRight =  frameGeneratorRight->getNextFrame(frameRight);
    
  }

  delete frameGeneratorLeft;
  delete frameGeneratorRight; 
  cv::destroyAllWindows();

  return 0;
}

