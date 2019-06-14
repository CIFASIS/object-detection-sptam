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

#include "CameraFrameGenerator.h"
#include "stdio.h"
#include "../libcam/libcam.h"
#include "opencv2/core/version.hpp"

CameraFrameGenerator::CameraFrameGenerator(const std::string& deviceName, int width, int height, int fps, int exposure)
{
  camera_ = new Camera(deviceName.c_str(), width, height, fps);
  camera_->setExposure(exposure);
  camera_->Update();

  frame_ = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
}

CameraFrameGenerator::~CameraFrameGenerator()
{
  delete camera_;
  cvReleaseImage( &frame_ );
}

bool CameraFrameGenerator::getNextFrame(cv::Mat& frame)
{
  camera_->Update( (unsigned int)1 );

  camera_->toIplImage( frame_ );

  #if CV_MAJOR_VERSION == 2
    frame = cv::Mat( frame_ );
  #elif CV_MAJOR_VERSION == 3
    frame = cv::cvarrToMat( frame_ );
  #endif

  return true;
}
