#pragma once

#include "../Map.hpp"

/// Result of a detection
struct DetectionMatch
{
  /// Detection status.
  bool status;
  /// Query id
  size_t query;
  /// Matched id if loop detected, otherwise, best candidate 
  size_t match;
  /// Matched score
  double score;
  
  /** Checks if the loop was detected */
  inline bool detection() const
  { return status; }
};

class LCDetector
{
  public:
    virtual ~LCDetector(){}
    virtual DetectionMatch detectloop(const sptam::Map::SharedKeyFrame& stereo_frame) = 0;
};
