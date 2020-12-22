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

#include "FeatureExtractorThread.hpp"

#ifdef SHOW_PROFILING
  #include "utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

FeatureExtractorThread::FeatureExtractorThread(
  const cv::Mat& image,
  cv::FeatureDetector& featureDetector,
  cv::DescriptorExtractor& descriptorExtractor,
  size_t nFeatures
)
  : image_( image )
  , featureDetector_( featureDetector )
  , descriptorExtractor_( descriptorExtractor )
  , nFeatures_(nFeatures)

{
  featureExtractorThread_ = std::thread(&FeatureExtractorThread::Extract, this);
}

#define PROFILE_INTERNAL 0 /* leave this disabled so that logging does not slow down parallel feature extraction */

void FeatureExtractorThread::Extract()
{
  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    sptam::Timer t_detect, t_extract;
    t_detect.start();
  #endif

  featureDetector_.detect(image_, keyPoints_);

  assert( not keyPoints_.empty() );

  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    t_detect.stop();
    t_extract.start();
  #endif

  if (nFeatures_ > 0 && keyPoints_.size() > nFeatures_)
    keyPoints_.resize(nFeatures_);

  descriptorExtractor_.compute(image_, keyPoints_, descriptors_);

  #if defined(SHOW_PROFILING) && PROFILE_INTERNAL
    t_extract.stop();
    WriteToLog(" tk FeatureDetection: ", t_detect.elapsed());
    WriteToLog(" tk DescriptorExtraction: ", t_extract.elapsed());
  #endif
}
