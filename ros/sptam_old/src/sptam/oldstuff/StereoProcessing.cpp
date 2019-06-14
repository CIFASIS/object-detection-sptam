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
#include "StereoProcessing.hpp"
#include "utils/projective_math.hpp"

#include <iostream>

bool ComputeCorrectMatches(
  const std::vector<std::vector<cv::KeyPoint> >& keypoints,
  const std::vector<cv::Mat>& descriptors,
  const std::vector<cv::DMatch>& matches,
  std::vector<std::vector<cv::KeyPoint> >& correctKeypoints,
  std::vector<cv::Mat>& correctDescriptors,
  std::vector<cv::DMatch>& correctMatches
)
{
  unsigned int totalMatches = matches.size();
  uint thresholdTotalMatches = 0;
  if (totalMatches > thresholdTotalMatches) {
    // Estimation of fundamental matrix using RANSAC algorithm
    std::vector<cv::Point2d> pointsLeft(totalMatches);
    std::vector<cv::Point2d> pointsRight(totalMatches);
    std::vector<uchar> status(totalMatches);

    // initialize the points here to pass to findFundamentalMat()

    for(unsigned int i = 0; i < totalMatches; ++i) {
      pointsLeft[i] = keypoints[0][matches[i].queryIdx].pt;
      pointsRight[i] = keypoints[1][matches[i].trainIdx].pt;
    }

    cv::Mat fundamentalMatrix = cv::findFundamentalMat(pointsLeft, pointsRight, cv::FM_RANSAC, 1, 0.99, status);

    std::cout << "Fundamental Matrix: " << fundamentalMatrix << std::endl;

    int correctKeypointLastIndex = 0;
    for(unsigned int i = 0; i < totalMatches; ++i) {
      if (status[i] == 1){
        cv::DMatch match = cv::DMatch(correctKeypointLastIndex,correctKeypointLastIndex, matches[i].distance);
        correctMatches.push_back(match);
        correctKeypoints[0].push_back(keypoints[0][matches[i].queryIdx]);
        correctKeypoints[1].push_back(keypoints[1][matches[i].trainIdx]);
        correctDescriptors.push_back(descriptors[0].row(matches[i].queryIdx));
        correctKeypointLastIndex++;
      }
    }
    return true;
  }
  else {
    return false;
  }
}

std::vector<cv::DMatch> FilterMatchesByF(
  const cv::Matx33d& fundamentalMatrix,
  const std::vector<cv::DMatch>& matches,
  const std::vector<cv::KeyPoint>& keypoints1,
  const std::vector<cv::KeyPoint>& keypoints2,
  double epipolarDistanceThreshold,
  double matchingDistanceThreshold
)
{
  std::vector<cv::DMatch> goodMatches;

  for ( auto match : matches ) {

    cv::Point2d proj1 = keypoints1[ match.queryIdx ].pt;
    cv::Point2d proj2 = keypoints2[ match.trainIdx ].pt;

    cv::Vec3d x1 = toHomo( proj1 );
    cv::Vec3d x2 = toHomo( proj2 );

    // Del libro Multiple View Geometry page: 287
    // e_t e = (x_t * F * x)^2
    double error_tError = x2.dot( fundamentalMatrix * x1);
    error_tError = error_tError * error_tError;

    // ((F x)^2)[0] + ((F x)^2)[1] + ((F_t x')^2)[0] + ((F_t x')^2)[1]
    cv::Vec3d Fx1 = (fundamentalMatrix * x1);
    cv::Vec3d Fx2 = (fundamentalMatrix.t() * x2);

    double JJ_t =
      Fx1[0] * Fx1[0] + Fx1[1] * Fx1[1] +
      Fx2[0] * Fx2[0] + Fx2[1] * Fx2[1];

    // JJ_t can not be zero
    if (JJ_t < 0.00000001) {
      std::cerr << "JJt = 0" << std::endl;
      continue;
    }

    double cost = error_tError / JJ_t;

    if (std::abs( cost ) <= epipolarDistanceThreshold &&
        match.distance < matchingDistanceThreshold) {
      goodMatches.push_back( match );
    }
  }

  return goodMatches;
}
