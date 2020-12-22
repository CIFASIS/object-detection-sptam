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

#include "MapMaker.hpp"
#include "match_to_points.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"
#include "utils/cv2eigen.hpp"
#include <boost/range/adaptor/indirected.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
#endif

#ifdef SHOW_PROFILING
  #include "utils/log/Profiler.hpp"
  #include "utils/log/ScopedProfiler.hpp"
#endif // SHOW_PROFILING

#define INITIAL_POINT_COVARIANCE Eigen::Matrix3d::Identity() * 1e-4

MapMaker::MapMaker(sptam::Map& map, const Parameters& params)
  : map_( map )
  , rowMatcher_( params.matchingDistanceThreshold, params.descriptorMatcher, params.epipolarDistanceThreshold )
  , local_keyframe_window_( params.nKeyFramesToAdjustByLocal ), params_( params )
{}

void MapMaker::InterruptBA()
{
  bundleDriver_.Break();
}

void MapMaker::CleanupMap(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames)
{
  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" ba handleBadPoints: ");
  #endif

  // Find the set of MapPoints watch from the given keyframes (This finding already did it in BA)
  // and select the bad ones
  sptam::Map::SharedMapPointSet bad_points; // use a set to avoid same MapPoints

  for ( sptam::Map::SharedKeyFrame& keyFrame : keyFrames )
    for ( sptam::Map::SharedMeas& meas : keyFrame->measurements() )
    {
      sptam::Map::SharedPoint& mapPoint = meas->mapPoint();

      if ( mapPoint->IsBad() )
        bad_points.insert( mapPoint );
    }

  // Remove the bad MapPoints
  {
    #ifdef SHOW_PROFILING
      sptam::Timer t_remove_bad_points_lock;
      t_remove_bad_points_lock.start();
    #endif

    boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);

    #ifdef SHOW_PROFILING
      t_remove_bad_points_lock.stop();
      WriteToLog(" ba remove_bad_points_lock: ", t_remove_bad_points_lock);
    #endif

    for ( const sptam::Map::SharedPoint& mapPoint : bad_points )
      map_.RemoveMapPoint( mapPoint );

    // Remove Keyframes with not enough measurements
    // TODO: Gaston: Loop Closing doesnt support removing KFs from map yet.
    //~ RemoveBadKeyFrames( keyFrames );
  }
}

sptam::Map::SharedKeyFrame MapMaker::AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements)
{
  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" ba totalba: ");
  #endif

  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

  // Create new 3D points from unmatched features,
  // and save them in the local tracking map.
  createNewPoints( keyFrame );

  // Load matched map point measurements into the new keyframe.
  // the point could have expired in the meantime, so check it.
  for ( auto& match : measurements )
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );

  /*#ifdef SHOW_PROFILING
    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif*/

  // GENERAL MAINTAINANCE

  // Refine Newly made points (the ones added from stereo matches
  // when the last keyframe came in)

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer2(" ba refind_newly_made: ");
  #endif

  std::list<sptam::Map::SharedPoint> aux_newpoints = getPointsCretaedBy( *keyFrame );
  /*int nFound = */ReFindNewlyMade( ListIterable<sptam::Map::SharedKeyFrame>::from( local_keyframe_window_ ), ListIterable<sptam::Map::SharedPoint>::from( aux_newpoints ) );

  #ifdef SHOW_PROFILING
  }
  #endif

  local_keyframe_window_.push( keyFrame );

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer2(" ba local: ");
  #endif

  BundleAdjust( ListIterable<sptam::Map::SharedKeyFrame>::from( local_keyframe_window_ ) );

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef USE_LOOPCLOSURE
  /* Notifying newly added keyframe to Loop Closure service */
  if(loopclosing_ != nullptr)
    loopclosing_->addKeyFrame(keyFrame);
  #endif

  // Remove bad points marked by BA

  CleanupMap( ListIterable<sptam::Map::SharedKeyFrame>::from( local_keyframe_window_ ) );

  return keyFrame;
}

void MapMaker::addStereoPoints(
  /*const */sptam::Map::SharedKeyFrame& keyFrame, std::vector<cv::Point3d>& points,
  std::vector<cv::KeyPoint>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
  std::vector<cv::KeyPoint>& featuresRight, std::vector<cv::Mat>& descriptorsRight
)
{
  // TODO revisar si se puede refinar un poco el pedido de lock

  #ifdef SHOW_PROFILING
    sptam::Timer t_lock_add_points;
    t_lock_add_points.start();
  #endif

  boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);

  #ifdef SHOW_PROFILING
    t_lock_add_points.stop();
    WriteToLog(" tk lock_add_points: ", t_lock_add_points);
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" tk add_points: ");
  #endif

  Eigen::Vector3d frame_position = keyFrame->GetPosition();
  forn ( i, points.size() )
  {

    Eigen::Vector3d normal = cv2eigen( points[ i ] ) - frame_position ;
    normal.normalize();

    sptam::Map::SharedPoint mapPoint = map_.AddMapPoint( MapPoint( cv2eigen( points[ i ] ), normal, descriptorsLeft[ i ], INITIAL_POINT_COVARIANCE ) );

    Measurement measurement(Measurement::SRC_TRIANGULATION, featuresLeft[i], descriptorsLeft[i], featuresRight[i], descriptorsRight[i]);

    map_.addMeasurement( keyFrame, mapPoint, measurement );

    // increase measurement counter in point
    mapPoint->IncreaseMeasurementCount();
  }

  #ifdef SHOW_PROFILING
  }
  #endif
}

void MapMaker::createNewPoints(/*const */sptam::Map::SharedKeyFrame& keyFrame)
{
  std::vector<cv::Point3d> points;
  std::vector<cv::KeyPoint> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" tk triangulate_points: ");
  #endif

  keyFrame->TriangulatePoints(
    rowMatcher_, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );

  #ifdef SHOW_PROFILING
  }
    WriteToLog(" tk created_new_points: ", points.size());
  #endif

  addStereoPoints(
    keyFrame, points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );
}

// dummy function in sequential mode (there is no safe window)
bool MapMaker::isSafe(sptam::Map::SharedKeyFrame keyframe) {
  return true;
}

sptam::Map::SharedKeyFrameSet MapMaker::getSafeCovisibleKFs(sptam::Map::SharedKeyFrameSet& baseKFs)
{
  sptam::Map::SharedKeyFrameSet covisibleKFs;

  for ( const sptam::Map::SharedKeyFrame& keyFrame : baseKFs )
    for (auto& kv : keyFrame->covisibilityKeyFrames())
    {
      const sptam::Map::SharedKeyFrame& covisible_keyframe = kv.first;

      if( !baseKFs.count( covisible_keyframe ) )
        if ( isSafe( covisible_keyframe ) )
          covisibleKFs.insert( covisible_keyframe );
    }

  return covisibleKFs;
}

bool MapMaker::BundleAdjust(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames)
{
  sptam::Map::SharedKeyFrameSet sAdjustSet, sFixedSet;

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_select: ");
  #endif

  for ( sptam::Map::SharedKeyFrame& keyFrame : keyFrames )
    if ( not keyFrame->isFixed() )
      sAdjustSet.insert( keyFrame );

  /* Any other keyframe inside the safe window that measure above points shared, will be used as fixed keyframe
   * Gaston: Loop Closure safe window its defined in the multi-threaded version through sincronization messages  */
  sFixedSet = getSafeCovisibleKFs( sAdjustSet );

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_load: ");
  #endif

  //std::cout << "BA Local: kf adj: " << sAdjustSet.size() << " kf fix: " <<  sFixedSet.size() << " points: " << points.size() << std::endl;

  // load the data into the bundle adjuster
  // No need to take a lock for the points, since the points are
  // aquired from the keyframe measurements and not the Map collection.
  ConstIterable<sptam::Map::SharedKeyFrame> aux1 = sptam::Map::Graph::createIterable( sAdjustSet );
  ConstIterable<sptam::Map::SharedKeyFrame> aux2 = sptam::Map::Graph::createIterable( sFixedSet );
  bundleDriver_.SetData(aux1, aux2);

  #ifdef SHOW_PROFILING
  }
  #endif

  //std::cout << "EJECUTANDO BA LOCAL:" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
    sptam::Timer t_adjust;
    t_adjust.start();
  #endif

  bool was_completed = bundleDriver_.Adjust( params_.maxIterationsLocal );

  #ifdef SHOW_PROFILING
    t_adjust.stop();
    WriteToLog(" ba local_adjust: ", t_adjust);
  #endif

  //std::cout << "DESPUES DE BA LOCAL" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_save_points: ");
  #endif

  // save data even if BA was interrupted
  /* NOTE: Gaston: No need for asking map lock as MapPoints have internal locking */
  bundleDriver_.SavePoints();

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_save_cameras: ");
  #endif

  // No lock is required: The keyframes have an internal lock for their modification
  bundleDriver_.SaveCameras();

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_handle_bad: ");
  #endif

  if ( was_completed )
  {
    std::list< sptam::Map::SharedMeas > aux = bundleDriver_.GetBadMeasurements();
    RemoveMeasurements( ListIterable<sptam::Map::SharedMeas>::from( aux ) );
  }
  else
    std::cout << "BA LOCAL NO CONVERGIO" << std::endl;

  bundleDriver_.Clear();

  #ifdef SHOW_PROFILING
  }
  #endif

  return was_completed;
}

std::list<sptam::Map::SharedPoint> MapMaker::getPointsCretaedBy(sptam::Map::KeyFrame& keyFrame)
{
  std::list<sptam::Map::SharedPoint> newMapPoints;

  for( const auto& measurement : keyFrame.measurements() )
    if ( measurement->GetSource() == Measurement::SRC_TRIANGULATION )
      newMapPoints.push_back( measurement->mapPoint() );

  return newMapPoints;
}

bool MapMaker::isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  return not mapPoint->IsBad() and keyFrame.canView( *mapPoint );
}

/**
 * Helper function. Filters points that could be
 * potential matches for a frame.
 */
std::list< sptam::Map::SharedPoint > MapMaker::filterUnmatched(const sptam::Map::KeyFrame& keyFrame, Iterable<sptam::Map::SharedPoint>& mapPoints)
{
  std::list< sptam::Map::SharedPoint > filtered_points;

  for ( sptam::Map::SharedPoint& mapPoint : mapPoints )
    if ( isUnmatched( keyFrame, mapPoint ) )
      filtered_points.push_back( mapPoint );

  return filtered_points;
}

size_t MapMaker::ReFindNewlyMade(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames, Iterable<sptam::Map::SharedPoint>&& new_points)
{
  // Is there new points?
  if( new_points.empty() )
    return 0;

  size_t nFound = 0;

  for( sptam::Map::SharedKeyFrame& keyFrame : keyFrames)
  {
    std::list< sptam::Map::SharedPoint > filtered_points = filterUnmatched(*keyFrame, new_points);

    for( sptam::Map::SharedPoint& mapPoint : filtered_points )
      mapPoint->IncreaseProjectionCount();

    std::list<Match> matches = matchToPoints(
      *keyFrame, ListIterable<sptam::Map::SharedPoint>::from( filtered_points )
      , params_.descriptorMatcher, params_.matchingNeighborhoodThreshold
      , params_.matchingDistanceThreshold, Measurement::SRC_REFIND
    );

    for (Match& match : matches) {
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );
      match.mapPoint->IncreaseMeasurementCount(); // increase measurement counter of mapPoint
    }

    nFound += matches.size();
  }

  return nFound;
}

void MapMaker::RemoveMeasurements(Iterable<sptam::Map::SharedMeas>&& measurements)
{
  for ( sptam::Map::SharedMeas& meas : measurements ) {
    meas->mapPoint()->IncreaseOutlierCount();
    map_.removeMeasurement( meas );
  }
}

/*void MapMaker::RemoveBadKeyFrames(const ConstIterable<sptam::Map::SharedKeyFrame>& keyFrames)
{
  for ( const sptam::Map::SharedKeyFrame& keyFrame : keyFrames ) {
    //    std::cout << "KF: " << keyFrame.GetId() << " meas: " << keyFrame.measurements().size() << std::endl;
    // If keyframe is considered "bad"
    if ( keyFrame->measurements().size() < MIN_NUM_MEAS ) {
      map_.RemoveKeyFrame( keyFrame );
    }
  }
}*/
