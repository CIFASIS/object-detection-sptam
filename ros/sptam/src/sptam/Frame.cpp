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

#include "Frame.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"
#include "utils/timer.h"
#include "utils/log/Profiler.hpp"

inline std::vector<cv::Point2d> project(const cv::Matx34d& projection, const std::vector<cv::Point3d>& points)
{
  std::vector<cv::Point2d> ret;
  ret.reserve( points.size() );

  for ( auto point : points )
    ret.push_back( project( projection, point ) );

  return ret;
}

Frame::Frame(const Camera& camera, const ImageFeatures& imageFeatures)
  : camera_( camera ), imageFeatures_( imageFeatures )
{}

#define PROFILE_INTERNAL 1

void Frame::FindMatches(
  const std::vector<cv::Point3d>& points,
  const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold,
  std::map<size_t, MEAS>& measurements
) const
{

#if SHOW_PROFILING && PROFILE_INTERNAL
  sptam::Timer t_project;
  t_project.start();
#endif

  std::vector<cv::Point2d> featurePredictions = project( camera_.GetProjection(), points );

#if SHOW_PROFILING && PROFILE_INTERNAL
  t_project.stop();
  sptam::Timer t_match;
  t_match.start();
#endif

  forn ( i, featurePredictions.size() ) {

    cv::Point2d& point = featurePredictions[ i ];
    cv::Mat descriptor = descriptors[ i ];

    int index = imageFeatures_.FindMatch(point, descriptor, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);

    if ( 0 <= index ) {

      imageFeatures_.SetMatchedKeyPoint( index );

      const cv::KeyPoint& matchedKeypoint = imageFeatures_.GetKeypoint( index );

      // A match is added to the result
      MEAS meas;
      meas.keypoint = matchedKeypoint;
      meas.descriptor = imageFeatures_.GetDescriptor( index );

      measurements.insert( std::pair<size_t,MEAS>(i, meas) );
    }
  }

#if SHOW_PROFILING && PROFILE_INTERNAL
  t_match.stop();
  WriteToLog(" xx FindMatchesFrame-project: ", t_project);
  WriteToLog(" xx FindMatchesFrame-match: ", t_match);
#endif
}

bool Frame::FindMatch(
  const cv::Point3d& point,
  const cv::Mat descriptor,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold,
  MEAS& meas
) const
{
  // Project Map Points to the keyFrame
  cv::Point2d prediction = project(camera_.GetProjection(), point);

  int index = imageFeatures_.FindMatch(prediction, descriptor, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);

  if ( 0 <= index ) {

    imageFeatures_.SetMatchedKeyPoint( index );

    const cv::KeyPoint& matchedKeypoint = imageFeatures_.GetKeypoint( index );

    // A match is added to the result
    meas.keypoint = matchedKeypoint;
    meas.descriptor = imageFeatures_.GetDescriptor( index );

    return true;
  } else {
    return false; // no match was found
  }
}
