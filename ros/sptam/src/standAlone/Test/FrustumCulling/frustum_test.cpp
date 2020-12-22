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

#include "utils/macros.hpp"
#include "Matching/FrustumCulling.h"
#include "utils/quaternion_math.hpp"

#include <random>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointXYZ toPCL(cv::Point3d point)
{
  pcl::PointXYZ point_pcl;
  point_pcl.x = point.x;
  point_pcl.y = point.y;
  point_pcl.z = point.z;
  return point_pcl;
}

int main(int argc, char *argv[])
{
  CameraPose cameraPose(cv::Vec3d(-200, -200, -200), anglesToQuaternion(cv::Vec3d(1.5, 0.5, 1.0)));

  double horizontalFOV = 100;
  double verticalFOV = 60;

  double nearPlaneDist = 0.1;
  double farPlaneDist = 600;

  FrustumCulling frustum(
    cameraPose, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist
  );

  //~ uint32_t PCL_BLUE = (static_cast<uint32_t>(0x0) << 16 | static_cast<uint32_t>(0x0) << 8 | static_cast<uint32_t>(0xFF));
  //~ uint32_t PCL_RED = (static_cast<uint32_t>(0xFF) << 16 | static_cast<uint32_t>(0x0) << 8 | static_cast<uint32_t>(0x0));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(-1000, 1000);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

  // generate some random points in the nearby space
  forn(i, 1000)
  {
    cv::Point3d point(dist(gen), dist(gen), dist(gen));
    //~ std:: cout << point << std::endl;

    pcl::PointXYZRGB point_pcl;

    if ( frustum.Contains( point ) )
      point_pcl = pcl::PointXYZRGB(255, 0, 0);
    else
      point_pcl = pcl::PointXYZRGB(0, 0, 255);

    point_pcl.x = point.x;
    point_pcl.y = point.y;
    point_pcl.z = point.z;

    //~ if (  )
    //~ point_pcl.rgb = *reinterpret_cast<float*>(&PCL_RED);
    //~ point_pcl.rgb = *reinterpret_cast<float*>(&PCL_BLUE);

    cloud->points.push_back( point_pcl );
  }

  pcl::visualization::PCLVisualizer viewer("3D Viewer");

  viewer.addCoordinateSystem(100.0);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer.addPointCloud<pcl::PointXYZRGB>( cloud, rgb, "cloud" );

  // dram camera pose
  {
    cv::Vec3d position = cameraPose.GetPosition();
    cv::Vec4d orientation = cameraPose.GetOrientationQuaternion();
    Eigen::Affine3f transformation(
      Eigen::Translation<float,3>(position[0],position[1], position[2]) *
      Eigen::Quaternionf(orientation[0], orientation[1], orientation[2], orientation[3])
    );
    viewer.addCoordinateSystem(100.0, transformation);
  }

  cv::Point3d bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner;
  frustum.GetFarPlaneCorners(bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner);

  // draw frustum
  pcl::PointXYZ  origin_pcl = toPCL( frustum.GetPosition() );
  pcl::PointXYZ  bottomLeftCorner_pcl = toPCL( bottomLeftCorner );
  pcl::PointXYZ  bottomRightCorner_pcl = toPCL( bottomRightCorner );
  pcl::PointXYZ  topLeftCorner_pcl = toPCL( topLeftCorner );
  pcl::PointXYZ  topRightCorner_pcl = toPCL( topRightCorner );

  viewer.addLine(origin_pcl, bottomLeftCorner_pcl, 0, 0, 1.0, "1");
  viewer.addLine(origin_pcl, bottomRightCorner_pcl, 0, 0, 1.0, "2");
  viewer.addLine(origin_pcl, topLeftCorner_pcl, 0, 0, 1.0, "3");
  viewer.addLine(origin_pcl, topRightCorner_pcl, 0, 0, 1.0, "4");

  viewer.addLine(bottomLeftCorner_pcl, bottomRightCorner_pcl, 0, 0, 1.0, "5");
  viewer.addLine(bottomRightCorner_pcl, topRightCorner_pcl, 0, 0, 1.0, "6");
  viewer.addLine(topRightCorner_pcl, topLeftCorner_pcl, 0, 0, 1.0, "7");
  viewer.addLine(topLeftCorner_pcl, bottomLeftCorner_pcl, 0, 0, 1.0, "8");

  // Set visualizer settings

  //~ viewer.initCameraParameters();
  viewer.setSize(800, 800);

  //--------------------
  // -----Main loop-----
  //--------------------

  while ( not viewer.wasStopped() )
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return EXIT_SUCCESS;
}
