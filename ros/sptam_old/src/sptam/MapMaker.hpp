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

#include "Map.hpp"
#include "loopclosing/LoopClosing.hpp"
#include "Match.hpp"
#include "BundleDriver.hpp"
#include "RowMatcher.hpp"
#include "sptamParameters.hpp"
#include "utils/fixed_queue.hpp"

// Minimum number of measurements required for a keyframe
// to be considered "good". If the number of measurements drops
// below this threshold, the keyframe is erased from the map.
// TODO esto debería ser un parámetro del programa creo.
// Lo pongo acá porque me da fiaca, perdón :( (thomas)
#define MIN_NUM_MEAS 10

/**
 * 
 */
class MapMaker
{
  public:

    MapMaker(sptam::Map& map, const Parameters& params);

    /**
     * Add a key-frame to the map. Called by the tracker.
     * The tracker entry point for adding a new keyframe;
     * the tracker thread doesn't want to hang about, so
     * just dumps it on the top of the mapmaker's queue to
     * be dealt with later, and return.
     */
    virtual sptam::Map::SharedKeyFrame AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements);

    void addStereoPoints(
      /*const */sptam::Map::SharedKeyFrame& keyFrame, std::vector<cv::Point3d>& points,
      std::vector<cv::KeyPoint>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
      std::vector<cv::KeyPoint>& featuresRight, std::vector<cv::Mat>& descriptorsRight
    );

    #ifdef USE_LOOPCLOSURE
    void setLoopClosing(std::shared_ptr<LoopClosing>& lc)
    {loopclosing_ = lc;}
    #endif

    size_t getKFsToAdjustByLocal()
    {return params_.nKeyFramesToAdjustByLocal;}


    /**
     * Dummy function in sequential mode
     */
    virtual void Stop(){;}

  // Some functions are protected so they are reachable
  // by the threaded version of the MapMaker.
  protected:

    // TODO ver si se puede hacer privado el mapa
    sptam::Map& map_;

    #ifdef USE_LOOPCLOSURE
    /* Loop Closure service, will notify him when a new key frame its added. */
    std::shared_ptr<LoopClosing> loopclosing_;
    #endif

    RowMatcher rowMatcher_;

    typedef fixed_queue< sptam::Map::SharedKeyFrame > KeyFrameCache;

  // Maintenance functions:

    // Interrupt Glogal Bundle Adjustment
    void InterruptBA();

    // Peform a local bundle adjustment which only adjusts a selection of keyframes.
    bool BundleAdjust(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames);

    void createNewPoints(sptam::Map::SharedKeyFrame& keyFrame);

    // Get Triangulated MapPoints from a given KeyFrame
    std::list<sptam::Map::SharedPoint> getPointsCretaedBy(sptam::Map::KeyFrame& keyFrame);

    /**
     * This tells if refind should try to match a certain point to a certain keyframe.
     * In the Sequential case, since the new_points are from a single keyframe
     * in each iteration, and that keyframe is not checked for refinds,
     * we can assume that the point was not matched before in the given list of keyframes.
     */
    virtual bool isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint);

    // When new map points are generated, they're only created from a stereo pair
    // this tries to make additional measurements in other KFs which they might
    // be in.
    // TODO se le puede cambiar el nombre a Refind (newly made depende del contexto),
    // también que devuelva la cantidad de encontrados y sacar el cout afuera.
    size_t ReFindNewlyMade(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames, Iterable<sptam::Map::SharedPoint>&& new_points);

    virtual void CleanupMap(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames);

    virtual void RemoveMeasurements(Iterable<sptam::Map::SharedMeas>&& measurements);

    void RemoveBadKeyFrames(const ConstIterable<sptam::Map::SharedKeyFrame>& keyFrames);

  // helper functions
  private:

    std::list< sptam::Map::SharedPoint > filterUnmatched(const sptam::Map::KeyFrame& keyFrame, Iterable<sptam::Map::SharedPoint>& mapPoints);

  private:

  /*** Private properties ***/

    BundleDriver bundleDriver_;

    KeyFrameCache local_keyframe_window_;

    virtual sptam::Map::SharedKeyFrameSet getSafeCovisibleKFs(sptam::Map::SharedKeyFrameSet& baseKFs);

    virtual bool isSafe(sptam::Map::SharedKeyFrame keyframe);


  protected:

  /*** Algorithm thresholds and parameters ***/

    Parameters params_;
};
