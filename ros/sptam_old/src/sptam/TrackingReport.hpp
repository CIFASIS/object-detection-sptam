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
 *           Matias Nitschẹ <mnitsche at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#pragma once

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
 #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/features2d.hpp>
#endif

#include <vector>
#include "ObjectMap.hpp"
#include "CameraPose.hpp"
#include "Map.hpp"
#include "Match.hpp"
#include "sptamParameters.hpp"

/**
 * Holds information regarding the result of a tracking operation
 * @todo to be extended to other report types
 */
class TrackingReport
{
  public:

    TrackingReport(const cv::Mat& left_in, const cv::Mat& right_in);

    /**
     * Draws side-by-side images of cameras with measurements and projections overlaid
     * @param before_refine if true, feature detection grid and unmatched keypoints are also drawn
     */
    void drawStereoFrame(const StereoFrame& frame,
                         const sptam::Map::SharedMapPointList& filtered_points,
                         const std::list<Match>& measurements,
                         const Parameters& params,
                         bool before_refine);

    void drawLeftFrame(const StereoFrame& frame,
                       const sptam::Map::SharedMapPointList& filtered_points,
                       const std::list<Match>& measurements,
                       const Parameters& params,
                       bool before_refine);
                       
    template<class T>
    void drawPoints(const StereoFrame& frame,
                    const T& points,
                    bool before_refine);


    void drawObjects(const StereoFrame& frame,
                     const sptam::ObjectMap::ObjectList& objects,
                     bool before_refine) ;


    cv::Mat leftFrame ;

    cv::Mat stereoFrameBeforeRefine, stereoFrameAfterRefine,
            leftFrameBeforeRefine, rightFrameBeforeRefine,
            leftFrameAfterRefine, rightFrameAfterRefine;

    CameraPose refinedCameraPose;

    sptam::Map::SharedMapPointList localMap;
    sptam::Map::SharedMapPointSet trackedMap;
    sptam::ObjectMap::ObjectList objectMap ;
    sptam::Map::SharedKeyFrameSet localKeyFrames;

    /**
     * Describes the possible outcomes of tracking
     */
    enum class State {
                       OK,                  /** tracking was succesful */
                       NOT_ENOUGH_POINTS    /** there were not enough points for tracking */
                     };
    State state;

    inline bool isOk() const
    { return state == TrackingReport::State::OK; }

    /**
     * Specifies what kind of output is expected of the tracker
     */
    enum DrawOutput
    {
      DRAW_NONE                  = 0,
      DRAW_BEFORE_REFINE_LEFT    = (1 << 0),
      DRAW_BEFORE_REFINE_RIGHT   = (1 << 1),
      DRAW_BEFORE_REFINE_STEREO  = (DRAW_BEFORE_REFINE_LEFT | DRAW_BEFORE_REFINE_RIGHT),
      DRAW_AFTER_REFINE_LEFT     = (1 << 2),
      DRAW_AFTER_REFINE_RIGHT    = (1 << 3),
      DRAW_AFTER_REFINE_STEREO   = (DRAW_AFTER_REFINE_LEFT | DRAW_AFTER_REFINE_RIGHT),
    };

    void enableDrawOutput(DrawOutput output);


  private:

    DrawOutput draw_output;

    const cv::Mat left_in_, right_in_;
};

