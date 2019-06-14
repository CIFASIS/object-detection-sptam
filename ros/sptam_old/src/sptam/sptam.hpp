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
#include "tracker_g2o.hpp"
#include "MapMakerThread.hpp"
#include "sptamParameters.hpp"

#include "PosePredictor.hpp"
#include "TrackingReport.hpp"

#include "ObjectMap.hpp"
#include <vector>


#ifdef USE_LOOPCLOSURE
#include "loopclosing/LoopClosing.hpp"
#include "loopclosing/LCDetector.hpp"
#endif

class SPTAM
{
  public:

    SPTAM(const RowMatcher& rowMatcher, const Parameters& params);

    #ifdef USE_LOOPCLOSURE
    SPTAM(
      const RowMatcher& rowMatcher, const Parameters& params,
      const std::shared_ptr<PosePredictor> posePredictor,
      std::unique_ptr<LCDetector>& loop_detector
    );
    #endif

    void init(/*const*/ StereoFrame& frame);

    /* TODO: Gaston: This getters serve as a proxy to the map object for the ros version,
     * this exposes const references to internal lists of the map. The ros node doesnt've access
     * to the map and reads this lists without any lock. This may produce undefined behaviour!. */
    inline bool isInitialized() const
    { return initialized_; }

    //return the total number of KeyFrames
    inline size_t nKeyFrames() 
            {  boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ ); 
               return map_.getKeyframes().size() ; 
            }
    //

    /**
     * Track current frame. If individual images are supplied, they are used to draw results of tracking.
     */
    void track(/*const*/ StereoFrame& frame,  TrackingReport& report);

    inline void stop()
    {
      mapMaker_.Stop();
      #ifdef USE_LOOPCLOSURE
      loopclosing_->stop();
      #endif
    }

    /* Quick tracking state implementation,
     * this way LoopClosing knows when its safe to correct trayectory */
    bool isTracking();

    /* Sincronization messages with loopclosure  */
    void setLoopCorrection(const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity());
    void pause();
    void unPause();
    bool isPaused();
    void stopAddingKeyframes();
    void resumeAddingKeyframes();
    bool isAddingKeyframesStopped();

    inline sptam::Map& GetMap()
    { return map_; }

    sptam::ObjectMap omap ;
    std::vector < sptam::Map::SharedKeyFrame > kf_list_ ;
    std::vector < CameraPose > delta_kf_list_ ;

    //std::vector < sptam::Object& > omap ;
    //
    inline size_t frames_since_last_kf() {
      return frames_since_last_kf_ ;
    }
    //
    inline const sptam::Map::SharedKeyFrame lastKeyFrame() {
      return lastKeyFrame_ ;
    }
    //
    inline const sptam::Map::SharedKeyFrame referenceKeyFrame() {
      return referenceKeyFrame_ ;
    }
  
 
  protected:

    sptam::Map map_;
    

    // the map builder interface
    // To run the mapper in a parallel thread, build an instance
    // of MapMakerThread.
    // If the run should be sequential, build an instance of MapMaker.
    #ifdef SINGLE_THREAD
      MapMaker mapMaker_;
    #else
      MapMakerThread mapMaker_;
    #endif

    // the tracker interface
    // To run the the tracker with g2o, build an instance
    // of tracker_g2o.
    // To run with PTAM based code, build an instance
    // of Tracker.
    tracker_g2o tracker_;

    // TODO esto se mantiene para ser utilizado nada mas
    // que en KeyFramePolicy. La policy debería ser una clase
    // en vez de una función y debería ocuparse de mantener esto.
    // De hecho, esta clase debería recibir como parametro
    // una politica de agregado de keyframes abstracta, pero vamos
    // a hacer los cambios de a poco.
    // Ahora sirve para pedir el mapa local. Eso quiere decir que hace falta,
    // sin embargo lo de arriba probablemente deberia cambiarse.
    /*const */sptam::Map::SharedKeyFrame lastKeyFrame_;

    sptam::Map::SharedKeyFrame referenceKeyFrame_;

    /* the set of points from the local map which matched to features in the last frame */
    sptam::Map::SharedMapPointSet tracked_map_;

    #ifdef USE_LOOPCLOSURE
    /* LoopClosing interface created using the loop detector passed. */
    std::shared_ptr<LoopClosing> loopclosing_; // pointer, as may or may-not be created

    /* PosePredictor defined */
    std::shared_ptr<PosePredictor> posePredictor_;
    #endif

    // This mutex must be use to protect isPause, isTracking, isAddingKeyframesStopped booleans and the error loop correction lc_T matrix.
    mutable std::mutex pause_mutex_;
    bool isTracking_ = false;
    bool isPaused_ = false;
    bool isAddingKeyframesStopped_ = false;
    void setTracking(bool);

    #ifdef USE_LOOPCLOSURE
    /* in case of a detected loop, the lc thread will report the acumulated error perceived
     * otherwise the Mat will be empty. This will serve as a correction for the posepredictor/motionmodel
     * prior received. The error its expresed as the delta tranformation between the two loop keyframes */
    Eigen::Isometry3d lc_T_ = Eigen::Isometry3d::Identity(); // 4x4 dimension!
    #endif

    Parameters params_;

    RowMatcher rowMatcher_;

  // some system statistics

    size_t frames_since_last_kf_;

  // helper functions

    /**
     * @brief select the map points that are relevant to a frame (i.e. the local map).
     *   Map points are filtered by frustum culling and by the
     *   angle-of-view of the last descriptor.
     *   TODO: esto se le deberia poder pedir al mapa, y el mapa
     *   los debería devolver de manera eficiente si fuese posible.
     */
    void filterPoints(const StereoFrame& frame, sptam::Map::SharedMapPointList &localMap, sptam::Map::SharedKeyFrameSet &localKeyFrames);

    bool shouldBeKeyframe(const StereoFrame& frame, const std::list<Match>& measurements);

  private:

    bool initialized_;
};
