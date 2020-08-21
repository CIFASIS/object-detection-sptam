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

#include "MapPoint.hpp"
#include "StereoFrame.hpp"
#include "Measurement.hpp"
#include "utils/CovisibilityGraph.hpp"
#include "utils/Iterable.hpp"

#include <boost/thread/shared_mutex.hpp>

namespace sptam
{

/**
 * This class represents the visibility map of the environment.
 * It is composed by 3D points in space, 6DOF camera views, and
 * the relationships between them, called measurements.
 */
class Map
{
  public:

    typedef CovisibilityGraph<StereoFrame, MapPoint, Measurement> Graph;

    typedef Graph::KeyFrame KeyFrame;
    typedef Graph::MapPoint Point;
    typedef Graph::Measurement Meas;

    typedef Graph::SharedKeyFrame SharedKeyFrame;
    typedef Graph::SharedMapPoint SharedPoint;
    typedef Graph::SharedMeasurement SharedMeas;

    typedef Graph::SharedPointIntPair SharedPointIntPair;
    typedef Graph::SharedMapPointMap SharedMapPointMap ;
    
    typedef Graph::SharedMapPointSet SharedMapPointSet;
    typedef Graph::SharedKeyFrameSet SharedKeyFrameSet;
    typedef Graph::SharedMeasurementSet SharedMeasurementSet;

    typedef Graph::SharedMapPointList SharedMapPointList;
    typedef Graph::SharedKeyFrameList SharedKeyFrameList;

//    typedef Graph::MapPointRefList PointRefs;

    SharedKeyFrame AddKeyFrame(const StereoFrame& frame)
    { return graph_.addKeyFrame( frame ); }

    void RemoveKeyFrame(const SharedKeyFrame& keyFrame)
    { // TODO: we have to take a lock!!!
      graph_.removeKeyFrame( keyFrame ); }

    SharedPoint AddMapPoint(const MapPoint& mapPoint)
    { return graph_.addMapPoint( mapPoint ); }

    void RemoveMapPoint(const SharedPoint& mapPoint)
    { graph_.removeMapPoint( mapPoint ); }

    void addMeasurement(SharedKeyFrame& keyFrame, SharedPoint& mapPoint, const Measurement& measurement)
    { graph_.addMeasurement( keyFrame, mapPoint, measurement ); }

    void removeMeasurement(const SharedMeas& measurement)
    { // TODO: we have to take a lock!!!
      graph_.removeMeasurement( measurement ); }

    inline void getLocalMap(const sptam::Map::SharedMapPointSet& trackedPoints, SharedMapPointSet& localMap, SharedKeyFrameSet& localKeyFrames, SharedKeyFrame& referenceKeyFrame )
    { graph_.getLocalMap( trackedPoints, localMap, localKeyFrames, referenceKeyFrame ); }

    const SharedKeyFrameList& getKeyframes() const
    { return graph_.getKeyframes(); }

    const SharedMapPointList& getMapPoints() const
    { return graph_.getMapPoints(); }

    //void RemoveBadPoints();

    /* Map points use an internal locking system, so they're thread-safe.
     * Yet the map and their keyframes arent, so we use an external mutex for
     * keyframes and map manipulation.
     * Tracking, Mapping and LoopClosing have to ask for the lock when:
     * Read-Lock: Get keyframes/mappoints, localMap or asking for any internal parameter.
     * Write-Lock: Adding points/frames, Remove points/frames, Modify any keyframe (even when using a reference outside the map).
     * TODO: Improve keyframe locking, give internal and external locking posibilities. */
    mutable boost::shared_mutex map_mutex_; // Map and Keyframe mutex

  private:

    Graph graph_;
};

} // namespace sptam
