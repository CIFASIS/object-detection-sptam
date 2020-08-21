#pragma once

#include "../LCDetector.hpp"

// DLoopDetector and DBoW2
#include "DBoW2.h"
#include "DLoopDetector.h"

class DLDBriefLoopDetector : public LCDetector
{
  public:
    struct Parameters : BriefLoopDetector::Parameters
    {
      Parameters(int height = 0, int width = 0, float frequency = 1, bool nss = true,
        float _alpha = 0.3, int _k = 0,
        DLoopDetector::GeometricalCheck geom = DLoopDetector::GEOM_NONE, int dilevels = 0)
        : BriefLoopDetector::Parameters(height, width, frequency, nss,
                                        _alpha, _k,
                                        geom, dilevels)
      {}

      static void loadFromYML(const std::string& file_path);
    };

    DLDBriefLoopDetector(const std::string& voc_file_path, const Parameters& params);

    DetectionMatch detectloop(const sptam::Map::SharedKeyFrame& stereo_frame);
  
  private:

    BriefVocabulary voc;
    BriefLoopDetector detector;
};
