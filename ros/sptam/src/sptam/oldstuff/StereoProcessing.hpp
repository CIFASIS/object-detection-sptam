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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

std::vector<cv::DMatch> FilterMatchesByF(
  const cv::Matx33d& fundamentalMatrix,
  const std::vector<cv::DMatch>& matches,
  const std::vector<cv::KeyPoint>& keypoints1,
  const std::vector<cv::KeyPoint>& keypoints2,
  double epipolarDistanceThreshold,
  double matchingDistanceThreshold
);

bool ComputeCorrectMatches(
  const std::vector<std::vector<cv::KeyPoint> >& keypoints,
  const std::vector<cv::Mat>& descriptors,
  const std::vector<cv::DMatch>& matches,
  std::vector<std::vector<cv::KeyPoint> >& correctKeypoints,
  std::vector<cv::Mat>& correctDescriptors,
  std::vector<cv::DMatch>& correctMatches
);
