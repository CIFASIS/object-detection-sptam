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

#include "ImageFeatures.hpp"
#include "utils/macros.hpp"

#include <functional>
#include <iostream>

bool compareResponse(int idx1, int idx2, const std::vector<cv::KeyPoint>& keyPoints)
{
  return keyPoints[ idx2 ].response < keyPoints[ idx1 ].response;
}

ImageFeatures::ImageFeatures(
  const cv::Size& image_size,
  const std::vector<cv::KeyPoint> keyPoints,
  const cv::Mat descriptors,
  const size_t MatchingCellSize)
  : MatchingCellSize_( MatchingCellSize )
  , nMatchingBucketsX_( ceil( image_size.width / (double) MatchingCellSize_ ) )
  , nMatchingBucketsY_( ceil( image_size.height / (double) MatchingCellSize_ ) )
  , hashedMatchingIndexes_( nMatchingBucketsY_ )
{
  // initialize Matching grid from size
  forn(i, nMatchingBucketsY_) {
    hashedMatchingIndexes_[ i ] = std::vector< std::vector<int> >( nMatchingBucketsX_ );
  }

  keyPoints_ = keyPoints;

  descriptors_ = descriptors;

  // Fill the spatial Matching hash grid with keypoint indexes

  forn(k, keyPoints_.size()) {

    iPair matchingHash = GetHash( keyPoints_[ k ].pt, MatchingCellSize_ );

    hashedMatchingIndexes_[ matchingHash.first ][ matchingHash.second ].push_back( k );
  }

  // Initially non keypoint is matched
  matchedKeyPoints_ = std::vector<bool>(keyPoints_.size(), false);
}


ImageFeatures::ImageFeatures(const ImageFeatures& imageFeatures)
  : matchedKeyPoints_(imageFeatures.matchedKeyPoints_)
  , keyPoints_(imageFeatures.keyPoints_)
  , MatchingCellSize_(imageFeatures.MatchingCellSize_)
  , nMatchingBucketsX_(imageFeatures.nMatchingBucketsX_)
  , nMatchingBucketsY_(imageFeatures.nMatchingBucketsY_)
  , nCreationBucketsX_(imageFeatures.nCreationBucketsX_)
  , nCreationBucketsY_(imageFeatures.nCreationBucketsY_)
  , hashedMatchingIndexes_(imageFeatures.hashedMatchingIndexes_)

{
  descriptors_ = imageFeatures.descriptors_.clone();
}



int ImageFeatures::FindMatch(
  const cv::Point2d& prediction,
  const cv::Mat& descriptor,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold
) const
{
  size_t radius = matchingNeighborhoodThreshold;

    // Get a list of indexes

  iPair hash = GetHash( prediction , MatchingCellSize_ );

  size_t fromY = std::max( (size_t)0, hash.first - radius );
  size_t toY = std::min( hash.first + radius + 1, nMatchingBucketsY_ );

  size_t fromX = std::max( (size_t)0, hash.second - radius );
  size_t toX = std::min( hash.second + radius + 1, nMatchingBucketsX_ );

  cv::Mat mask(cv::Mat::zeros(1, keyPoints_.size(), CV_8UC1));

  // search the features which fall in the neightbors cells
  forsn(i, fromY, toY) {
    forsn(j, fromX, toX) {
      for ( auto idx : hashedMatchingIndexes_[ i ][ j ] ) {

        // only try unmatched keypoints
        if ( not matchedKeyPoints_[ idx ] ) {
          mask.at<uchar>(0, idx) = true;
        }
      }
    }
  }

  std::vector<cv::DMatch> matches;
  descriptorMatcher.match(descriptor, descriptors_, matches, mask);

  /*
  std::vector< std::vector<cv::DMatch> > match_candidates;
  descriptorMatcher.knnMatch( descriptor, descriptors_, match_candidates, 2, mask);

  // was found a match?
  if (match_candidates.empty())
    return -1;

  std::vector<cv::DMatch>& matches = match_candidates[0];
  */

  // was found a match?
  if (matches.empty())
    return -1;

  // check if satisfy the matching distance threshold
  if (matches[0].distance > matchingDistanceThreshold)
    return -1;

  // check if the second match is close
//  if (matches.size() == 2) {
//    const double ratio = 1.01; // TODO: ver este parametro
//    if(ratio * matches[0].distance >  matches[1].distance)
//      return -1;
//  }

  return matches[0].trainIdx;
}

ImageFeatures::iPair ImageFeatures::GetHash(const cv::Point2d& key, const size_t cellSize) const
{
  return iPair( floor(key.y / (double) cellSize), floor(key.x / (double) cellSize) );
}

void ImageFeatures::GetUnmatchedKeyPoints( std::vector<cv::KeyPoint>& keyPoints,
                                           cv::Mat& descriptors,
                                           std::vector<size_t>& indexes) const
{
  // Get the matches which are not matched

  /* se reserva el espacio para el peor caso */
  keyPoints.reserve(keyPoints_.size());
  descriptors.reserve(keyPoints_.size());
  indexes.reserve(keyPoints_.size());

  forn (i, keyPoints_.size()) {
    if( not matchedKeyPoints_[ i ] ) {
      keyPoints.push_back( GetKeypoint( i ) );
      descriptors.push_back( GetDescriptor( i ) );
      indexes.push_back( i );
    }
  }
}
