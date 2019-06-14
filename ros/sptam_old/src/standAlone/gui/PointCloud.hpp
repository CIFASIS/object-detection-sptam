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

#include "../../sptam/Map.hpp"
#include "../../sptam/FrustumCulling.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


// mutex para thread del viewer
static boost::mutex updateModelMutex;

// class to visualizate the Point Cloud
class PointCloud {
 public:
  PointCloud(CameraPose& cameraPoseTracked, const sptam::Map& map,
             const double horizontalFOV = 0, const double verticalFOV = 0,
             const double nearPlaneDist = 0, const double farPlaneDist = 0);

  void operator()();

  void Stop();

  void SetCameraPose( CameraPose& cameraPoseTracked ) {
    cameraPoseTracked_ = cameraPoseTracked;
    updated = true;
  }

 private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  CameraPose& cameraPoseTracked_;
  const sptam::Map& map_;
  bool updated;


  // Frustum variables
  double horizontalFOV_;
  double verticalFOV_;
  double nearPlaneDist_;
  double farPlaneDist_;

  // Thread variable
  boost::thread thread_;
  bool stop_;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> create3DMapVisualizer(const sptam::Map &map,
                                                                           const double horizontalFOV,
                                                                           const double verticalFOV,
                                                                           const double nearPlaneDist,
                                                                           const double farPlaneDist);

void draw3DMapAndCurrentCameraPose(const CameraPose& cameraPoseTracked,
                                   const sptam::Map &map, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                   FrustumCulling& FrustumCulling);

// ------------------------------------------------------------------------------------------------------------

// ShowLine dibuja una linea entre dos puntos en el visualizador (es utilizada para dibujar el Frustum y la trayectoria de la camara)
void ShowLine(const Eigen::Vector3d &point1,
              const Eigen::Vector3d &point2,
              const double& red,
              const double& green,
              const double& blue,
              boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const std::string &id);
// ------------------------------------------------------------------------------------------------------------

void DrawPlane(const cv::Vec4f& plane, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const std::string &id = "plane");

// ------------------------------------------------------------------------------------------------------------

void RemoveCameraPoses(const std::vector<StereoFrame*>& keyFrames, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

void CreatePLYFile(const sptam::Map& map);
