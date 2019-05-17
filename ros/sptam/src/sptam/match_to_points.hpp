#include <list>
#include "Match.hpp"

// Used in MapMaker.cpp
std::list<Match> matchToPoints(
  const StereoFrame& frame,
  Iterable<sptam::Map::SharedPoint>&& mapPoints,
  const cv::Ptr<cv::DescriptorMatcher> descriptorMatcher,
  const size_t matchingNeighborhoodThreshold,
  const double matchingDistanceThreshold,
  const Measurement::Source source
);
