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

#include "../sptam/StereoFrame.hpp"
struct StereoImageFeatures {

  typedef std::unique_ptr<StereoImageFeatures> UniquePtr;

  StereoImageFeatures(
    const ImageFeatures& imgFeaturesLeft,
    const ImageFeatures& imgFeaturesRight,
    const cv::Mat& imgLeft,
    const cv::Mat& imgRight)
    : imageFeaturesLeft( imgFeaturesLeft )
    , imageFeaturesRight( imgFeaturesRight ) {

      imageLeft = imgLeft.clone();
      imageRight = imgRight.clone();

  }

  ImageFeatures imageFeaturesLeft;
  ImageFeatures imageFeaturesRight;

  cv::Mat imageLeft;
  cv::Mat imageRight;
};
