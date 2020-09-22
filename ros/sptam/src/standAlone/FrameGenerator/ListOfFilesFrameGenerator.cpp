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

#include "ListOfFilesFrameGenerator.h"

#include <iostream>
#include <boost/regex.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgcodecs.hpp>
  #include <opencv2/highgui.hpp>
#endif

void ExtractNumberOfFileName(const std::string& absolutePath, std::string& number)
{

  // Extract Frame file name without extension and absolute absolutePath
  unsigned int positionOfPointExtension = absolutePath.rfind('.');
  unsigned int positionOfSlashabsolutePath = absolutePath.rfind('/');
  std::string fileName = absolutePath.substr(positionOfSlashabsolutePath+1,positionOfPointExtension-positionOfSlashabsolutePath-1);

  // Extract timeStamp or number of image from name
  boost::regex timeStamp("\\d+[\\.]*\\d+");
  boost::match_results<std::string::const_iterator> results;
  if (boost::regex_search(fileName, results, timeStamp)) {
    number = results[0];  //Get contents of the second (...)
  }

//  std::cout << "TimeStamp:  "  << number << std::endl;
}

ListOfFilesFrameGenerator::ListOfFilesFrameGenerator(std::string fileName)
  : file_(fileName.c_str())
{}

ListOfFilesFrameGenerator::~ListOfFilesFrameGenerator()
{
  file_.close();
}

bool ListOfFilesFrameGenerator::getNextFrame(cv::Mat& frame)
{
  if (file_.eof()) {
    frame.create(0,0,CV_8U);
    return false;
  }

  std::string currentFrame;
  if (getline (file_, currentFrame))
  {
    frame = cv::imread(currentFrame.c_str());

    if (not frame.data)
      throw std::invalid_argument("Unable to read image: " + currentFrame);

    // Extract the timeStamp for rawseed dataset
    ExtractNumberOfFileName(currentFrame, currentFileName_);

  return true;
  }
  return false;
}
