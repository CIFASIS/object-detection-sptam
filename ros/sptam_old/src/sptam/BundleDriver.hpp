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

#include "g2o_driver.hpp"

#include <g2o/core/sparse_optimizer.h>

/**
 * The driver is an interface between the SPTAM framework and an external
 * bundle adjustment optimization routine.
 * optimizer setup and data format conversions are handled here.
 */
class BundleDriver
{
  public:

    /**
     * Add the data to be used in the next adjustment call ( Adjust )
     */
    void SetData(ConstIterable<sptam::Map::SharedKeyFrame>& adjustViews, ConstIterable<sptam::Map::SharedKeyFrame>& fixedViews);

    /**
     * Perform bundle adjustment for the given parameters.
     * return true if BA finish, false if it was interrupted
     *
     * @return
     *   return True if the process was explicitly interrupted by the user
     *   (using Break()). False otherwise.
     */
    bool Adjust(int maxIterations);

    /**
     * Load the parameters adjusted on the last adjustment call ( Adjust )
     */
    void SavePoints();
    void SaveCameras();

    /**
     * Handle Bad Measurements
     */
    std::list< sptam::Map::SharedMeas > GetBadMeasurements();

    /**
     * Interrupt the bundle adjustment if there is one in progress.
     * This function is (probably) called from another thread in which
     * the Adjust(...) function is running.
     */
    void Break();

    void Clear();

  private:

    G2ODriver g2o_driver_;

    std::vector<G2ODriver::Vertex*> point_vertices_;
    std::vector<G2ODriver::Vertex*> camera_vertices_;

    std::vector< sptam::Map::SharedMeas > measurements_;

    /**
     * Add Measurement to the Bundle using the id_maps_
     */
    void AddMeasToBundle();
};
