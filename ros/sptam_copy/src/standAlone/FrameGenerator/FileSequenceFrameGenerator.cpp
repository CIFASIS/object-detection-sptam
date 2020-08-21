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

#include "FileSequenceFrameGenerator.h"

#include <iostream>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgcodecs.hpp>
  #include <opencv2/highgui.hpp>
#endif

FileSequenceFrameGenerator::FileSequenceFrameGenerator(std::string path, size_t imageBeginIndex, size_t imageEndIndex)
  : path_(path), imageBeginIndex_( imageBeginIndex )
  , imageEndIndex_( imageEndIndex ), imageActualIndex_( imageBeginIndex )
{
  cv::glob(path_, filenames_);
}

bool FileSequenceFrameGenerator::getNextFrame(cv::Mat& frame)
{
  // if there is not more images, return false
  if (imageActualIndex_ > imageEndIndex_ or imageActualIndex_ == filenames_.size() ) {
    return false;
  }

  // get image path
  currentFilename_ = filenames_[imageActualIndex_];

  // read image
  frame = cv::imread( currentFilename_.c_str() );

  if (!frame.data) {
    // print error message when was not possbile to read the first image
    if (imageActualIndex_ == imageBeginIndex_) {
      std::cerr << "Unable to read image: " << currentFilename_ << std::endl;
    }
    return false;
  }

  // move image index to the next one
  imageActualIndex_++;

  return true;
}
