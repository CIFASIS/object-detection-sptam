#include "match_to_points.hpp"
#include "utils/macros.hpp"
#include "utils/cv2eigen.hpp"
#include "utils/timer.h"
#include "utils/log/Profiler.hpp"

std::list<Match> matchToPoints(
  const StereoFrame& frame,
  Iterable<sptam::Map::SharedPoint>&& mapPoints,
  const cv::Ptr<cv::DescriptorMatcher> descriptorMatcher,
  const size_t matchingNeighborhoodThreshold,
  const double matchingDistanceThreshold,
  const Measurement::Source source)
{
#if SHOW_PROFILING
  sptam::Timer t_find;
  t_find.start();
#endif

  std::vector<cv::Point3d> points;
  std::vector<cv::Mat> descriptors;

  points.reserve( mapPoints.size() );
  descriptors.reserve( mapPoints.size() );

  for ( const sptam::Map::SharedPoint& mapPoint : mapPoints ) {
    points.push_back( eigen2cv( mapPoint->GetPosition() ) );
    descriptors.push_back( mapPoint->GetDescriptor() );
  }

  std::map<size_t, MEAS> meas_left_pairs, meas_right_pairs;

  frame.FindMatches(
    points, descriptors,
    *descriptorMatcher,
    matchingNeighborhoodThreshold,
    matchingDistanceThreshold,
    meas_left_pairs, meas_right_pairs
  );

#if SHOW_PROFILING
  t_find.stop();
  WriteToLog(" XX findMatches-find: ", t_find);
  sptam::Timer t_process;
  t_process.start();
#endif

  std::list<Match> matches;

  // Esto es engorroso, pero eficiente
  size_t src_idx = 0;
  Iterator<sptam::Map::SharedPoint> it_src = mapPoints.begin();
  auto it_left = meas_left_pairs.begin();
  auto it_right = meas_right_pairs.begin();

  // iterate while there is something to be added to matches.
  while ( it_left != meas_left_pairs.end() or it_right != meas_right_pairs.end() )
  {
    // if both have the same source mapPoint, add a stereo measurement.
    if ( it_left != meas_left_pairs.end() and it_right != meas_right_pairs.end() and it_left->first == it_right->first )
    {
      // update src iterator and index
      forn(i, it_left->first - src_idx)
        it_src++;
      src_idx = it_left->first;

      const MEAS& meas_left = it_left->second;
      const MEAS& meas_right = it_right->second;

      // Create the Measurement
      Measurement measurement(source, meas_left.keypoint, meas_left.descriptor, meas_right.keypoint, meas_right.descriptor);
      matches.push_back( Match(*it_src, measurement) );

      ++ it_left;
      ++ it_right;
    }
    // if the next index to process is the left one.
    else if ( it_right == meas_right_pairs.end() or ( it_left != meas_left_pairs.end() and it_left->first < it_right->first ) )
    {
      const MEAS& meas_left = it_left->second;

      // update src iterator and index
      forn(i, it_left->first - src_idx)
        it_src++;
      src_idx = it_left->first;

      // Create the Measurement
      Measurement measurement(Measurement::LEFT, source, meas_left.keypoint, meas_left.descriptor);
      matches.push_back( Match(*it_src, measurement) );

      ++ it_left;
    }
    // if the next index to process is the right one.
    else
    {
      const MEAS& meas_right = it_right->second;

      // update src iterator and index
      forn(i, it_right->first - src_idx)
        it_src++;
      src_idx = it_right->first;

      Measurement measurement(Measurement::RIGHT, source, meas_right.keypoint, meas_right.descriptor);
      matches.push_back( Match(*it_src, measurement) );

      ++ it_right;
    }
  }

#if SHOW_PROFILING
  t_process.stop();
  WriteToLog(" XX findMatches-process: ", t_process);
#endif

  return matches;
}

//std::list<Measurement> matchToPoints(
//  const StereoFrame& frame,
//  const std::vector<MapPoint*>& mapPoints,
//  const cv::Ptr<cv::DescriptorMatcher> descriptorMatcher,
//  const size_t matchingNeighborhoodThreshold,
//  const double matchingDistanceThreshold,
//  const Measurement::Source_t source)
//{
//  std::vector<cv::Point3d> points;
//  std::vector<cv::Mat> descriptors;

//  points.reserve( mapPoints.size() );
//  descriptors.reserve( mapPoints.size() );

//  for ( auto mapPoint : mapPoints ) {
//    points.push_back( mapPoint->GetPosition() );
//    descriptors.push_back( mapPoint->GetDescriptor() );
//  }

//  std::map<int, MEAS> meas_left_pairs, meas_right_pairs;

//  frame.FindMatches(
//    points, descriptors,
//    *descriptorMatcher,
//    matchingNeighborhoodThreshold,
//    matchingDistanceThreshold,
//    meas_left_pairs, meas_right_pairs
//  );

//  std::list<Measurement> measurements;

//  for ( auto meas_left_pair : meas_left_pairs )
//  {
//    int index_mapPoint = meas_left_pair.first;
//    const MEAS& meas = meas_left_pair.second;

//    MapPoint* mapPoint = mapPoints[ index_mapPoint ];

//    // Update the descriptor of the MapPoint with the left image descriptor for stereo measurements
//    // TODO: aca haria falta pedir el lock
//    mapPoints[ index_mapPoint ]->SetDescriptor( meas.descriptor );

//    std::map<int,MEAS>::iterator it;
//    it = meas_right_pairs.find( index_mapPoint );

//    if ( it !=  meas_right_pairs.end() )
//    {
//      // Create the Measurement
//      std::vector<double> projection_stereo {meas.projection.x, meas.projection.y, it->second.projection.x};
//      Measurement measurement(mapPoint, projection_stereo, meas.descriptor);
//      measurement.source = source;
//      measurement.type = Measurement::STEREO;
//      measurements.push_back( measurement );

//      // remove stereo meas from right measurements
//      meas_right_pairs.erase(it);

//    } else {

//      // Create the Measurement
//      std::vector<double> projection_left {meas.projection.x, meas.projection.y};
//      Measurement measurement(mapPoint, projection_left, meas.descriptor);
//      measurement.source = source;
//      measurement.type = Measurement::LEFT;
//      measurements.push_back( measurement );
//    }
//  }

//  for ( auto meas_right_pair : meas_right_pairs )
//  {
//    int index_mapPoint = meas_right_pair.first;
//    const MEAS& meas = meas_right_pair.second;

//    // Update the descriptor of the MapPoint with the right image descriptor for only right measurements
//    // TODO: aca haria falta pedir el lock
//    mapPoints[ index_mapPoint ]->SetDescriptor( meas.descriptor );

//    // Create the Measurement
//    MapPoint* mapPoint = mapPoints[ index_mapPoint ];
//    std::vector<double> projection_right {meas.projection.x, meas.projection.y};
//    Measurement measurement(mapPoint, projection_right, meas.descriptor);
//    measurement.source = source;
//    measurement.type = Measurement::RIGHT;
//    measurements.push_back( measurement );
//  }

//  return measurements;
//}

