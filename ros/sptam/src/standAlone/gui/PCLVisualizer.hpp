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

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class PCLVisualizer
{
  public:

    PCLVisualizer(const std::string& name = "3D Viewer");

    ~PCLVisualizer();

    /**
     * Block until the PCL window is closed.
     */
    void wait();

    void addMapPoint(const Eigen::Vector3d& point);

    void addCamera(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist, const Eigen::Matrix6d& covariance);

  private:

    bool stop_;

    bool updated_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_;

    typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualizerPtr;
    PCLVisualizerPtr viewer_;

    boost::thread thread_;

  // helper functions

    /**
     * @brief Executes the main loop until a stop signal is given. Blocking.
     */
    void mainLoop();

    /**
     * @brief Creates and initializes the PCL viewer object.
     */
    PCLVisualizerPtr initViewer(const std::string& name);

    void drawLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, double r, double g, double b, const std::string& id);

    void drawFrustum(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist);

    void drawCovariance(const Eigen::Vector3d& position, const Eigen::Matrix6d& covariance);
};
