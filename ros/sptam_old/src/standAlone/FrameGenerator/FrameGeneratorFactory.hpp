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
#pragma once

#include "IFrameGenerator.h"
#include "CameraFrameGenerator.h"
#include "VideoFileFrameGenerator.h"
#include "ListOfFilesFrameGenerator.h"
#include "FileSequenceFrameGenerator.h"

IFrameGenerator* createFrameGenerator(
  const std::string& imagesSource,
  const std::string& imagesSourceType,
  const size_t imageBeginIndex = 0,
  const size_t imageEndIndex = std::numeric_limits<size_t>::max()
){
  if ( imagesSourceType.compare("vid") == 0 ) {
    return new VideoFileFrameGenerator( imagesSource );
  }
  else if ( imagesSourceType.compare("dir") == 0 ) {
    return new FileSequenceFrameGenerator( imagesSource, imageBeginIndex, imageEndIndex);
  }
  else if ( imagesSourceType.compare("list" ) == 0) {
    return new ListOfFilesFrameGenerator( imagesSource );
  }
  // TODO does not work anymore
  /*else if ( imagesSourceType.compare("cam") == 0 ) {
    frameGenerator = new CameraFrameGenerator(imagesSource.c_str(),
      cameraCalibration.imageWidth, cameraCalibration.imageHeight, 15, 100);
  }*/

  throw std::invalid_argument("Images source type is not correct");
}
