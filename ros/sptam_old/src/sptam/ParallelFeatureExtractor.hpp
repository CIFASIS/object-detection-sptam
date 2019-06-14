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

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
  #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
  #include <opencv2/xfeatures2d.hpp>
#endif

/**
 * Class to execute extract features in parallel
 */
class ParallelFeatureExtractor : public cv::ParallelLoopBody
{

  public:
    ParallelFeatureExtractor( cv::Mat* images,
                              cv::FeatureDetector* featureDetector,
                              cv::DescriptorExtractor* descriptorExtractor,
                              std::vector<cv::KeyPoint>* keyPoints,
                              cv::Mat* descriptors
                              )
                            : images_( images )
                            , featureDetector_( featureDetector )
                            , descriptorExtractor_( descriptorExtractor )
                            , keyPoints_( keyPoints )
                            , descriptors_( descriptors ){}

    virtual void operator()( const cv::Range &r ) const {
      register cv::Mat* images = images_ + r.start;
      register std::vector<cv::KeyPoint>* keyPoints = keyPoints_ + r.start;
      register cv::Mat* descriptors = descriptors_ + r.start;

      for (register int jf = r.start; jf != r.end; ++jf, ++images, ++keyPoints, ++descriptors)
      {
        // Compute Keypoints and descriptors

        featureDetector_->detect(*images, *keyPoints);

        descriptorExtractor_->compute(*images, *keyPoints, *descriptors);
      }
    }


  private:

    cv::Mat* images_;
    cv::FeatureDetector* featureDetector_;
    cv::DescriptorExtractor* descriptorExtractor_;

    cv::Mat* descriptors_;

    std::vector<cv::KeyPoint>* keyPoints_;

};
