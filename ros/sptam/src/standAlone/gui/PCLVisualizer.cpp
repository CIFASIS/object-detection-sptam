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
#include "PCLVisualizer.hpp"
#include "../../sptam/FrustumCulling.hpp"
#include "../../sptam/utils/covariance_ellipsoid.hpp"

void pointPickingActionCallBack (const pcl::visualization::PointPickingEvent& event);

pcl::PolygonMesh computeCameraMesh(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist, float r = 0., float g = 0., float b = 0., double s = 1.0);

pcl::PointXYZRGBA eigen2pcl(Eigen::Vector3d point, float r, float g, float b, float a);

////////////////////////////////////////////////////////////////////////
// Public functions

PCLVisualizer::PCLVisualizer(const std::string& name)
  : stop_( false ), updated_( false )
  , point_cloud_( new pcl::PointCloud<pcl::PointXYZRGBA>() )
  , viewer_( initViewer( name ) )
{
  thread_ = std::thread(&PCLVisualizer::mainLoop, this);
}

PCLVisualizer::~PCLVisualizer()
{
  stop_ = true;
  wait();
}

void PCLVisualizer::wait()
{
  thread_.join();
}

void PCLVisualizer::addMapPoint(const Eigen::Vector3d& point)
{
  // show only point which distance z is less than 600mm.
  // There are points that are far (60cm) from the camera that are
  // not visualized but are still used for SPTAM processing
  // this happens because the visualizer scales the cloud to contain
  // all the points and it is not possible to appreciate the cloud properly
  if ( /*600 < std::abs( point.x ) or 600 < std::abs( point.y ) or*/ 600 < std::abs( point[2] ) )
    return;

  // create PCL point
  pcl::PointXYZRGBA point_pcl;
  point_pcl.x = point[0];
  point_pcl.y = point[1];
  point_pcl.z = point[2];

  point_pcl.r = 0;
  point_pcl.g = 255;
  point_pcl.b = 0;

  point_cloud_->points.push_back( point_pcl );
  viewer_->updatePointCloud(point_cloud_, "map");

  updated_ = true;
}

void PCLVisualizer::addCamera(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist, const Eigen::Matrix6d& covariance)
{
  //~ pcl::PolygonMesh camera_mesh = computeCameraMesh(position, orientation, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist, 255, 255, 255);
  //~ viewer_->addPolygonMesh( camera_mesh );

  drawFrustum(position, orientation, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);

  drawCovariance(position, covariance);

  updated_ = true;
}

////////////////////////////////////////////////////////////////////////
// Private functions

void PCLVisualizer::mainLoop()
{
  while (!stop_ || viewer_->wasStopped())
  {
    if( updated_ ) {
      // TODO ...
      //~ draw3DMapAndCurrentCameraPose(cameraPoseTracked_, map_, viewer_, frustumCulling);
      updated_ = false;
    }

    viewer_->spinOnce( 100 );
  }
}

PCLVisualizer::PCLVisualizerPtr PCLVisualizer::initViewer(const std::string& name)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( name ) );

  // add point cloud to the viewer
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb( point_cloud_ );
  viewer->addPointCloud<pcl::PointXYZRGBA>(point_cloud_, rgb, "map");

  // Init other parameters
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.5f);

  // register point clicking (use PointPickingActionCallBack function callback)
  viewer->registerPointPickingCallback( pointPickingActionCallBack );

  return viewer;
}

void PCLVisualizer::drawLine(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, double r, double g, double b, const std::string& id)
{
  pcl::PointXYZ p1, p2;

  p1.x = static_cast<float>(point1[0]);
  p1.y = static_cast<float>(point1[1]);
  p1.z = static_cast<float>(point1[2]);

  p2.x = static_cast<float>(point2[0]);
  p2.y = static_cast<float>(point2[1]);
  p2.z = static_cast<float>(point2[2]);

  viewer_->addLine(p1, p2, r, g, b, id);
}

void PCLVisualizer::drawFrustum(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist)
{
  FrustumCulling frustum(position, orientation, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);

  Eigen::Vector3d bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner;
  frustum.GetFarPlaneCorners(bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner);

  drawLine(position, bottomLeftCorner, 255, 255, 255., "1");
  drawLine(position, bottomRightCorner, 255, 255, 255., "2");
  drawLine(position, topLeftCorner, 255, 255, 255., "3");
  drawLine(position, topRightCorner, 255, 255, 255., "4");

  drawLine(bottomLeftCorner, bottomRightCorner, 255, 255, 255, "5");
  drawLine(topLeftCorner, topRightCorner, 255, 255, 255, "6");
  drawLine(bottomLeftCorner, topLeftCorner, 255, 255, 255, "7");
  drawLine(bottomRightCorner, topRightCorner, 255, 255, 255, "8");
}

void PCLVisualizer::drawCovariance(const Eigen::Vector3d& position, const Eigen::Matrix6d& covariance)
{
  Ellipsoid3D ellipsoid = computeCovarianceEllipsoid(covariance.block(0, 0, 3, 3), PROB_99);
  
  const Eigen::Vector3d ax1_min = position + ellipsoid.len1 * ellipsoid.ax1;
  const Eigen::Vector3d ax1_max = position - ellipsoid.len1 * ellipsoid.ax1;

  const Eigen::Vector3d ax2_min = position + ellipsoid.len2 * ellipsoid.ax2;
  const Eigen::Vector3d ax2_max = position - ellipsoid.len2 * ellipsoid.ax2;

  const Eigen::Vector3d ax3_min = position + ellipsoid.len3 * ellipsoid.ax3;
  const Eigen::Vector3d ax3_max = position - ellipsoid.len3 * ellipsoid.ax3;

  drawLine(ax1_min, ax1_max, 255, 0, 0, "ellipsoid1");
  drawLine(ax2_min, ax2_max, 255, 0, 0, "ellipsoid2");
  drawLine(ax3_min, ax3_max, 255, 0, 0, "ellipsoid3");
}

////////////////////////////////////////////////////////////////////////
// Helper functions

pcl::PointXYZRGBA eigen2pcl(Eigen::Vector3d point, float r, float g, float b, float a)
{
  pcl::PointXYZRGBA p;

  p.x = point[0];
  p.y = point[1];
  p.z = point[2];

  p.r = r;
  p.g = g;
  p.b = b;
  p.a = a;

  return p;
}

void printPoint__(const Eigen::Vector3d& x)
{
  std::cout << "[" << x[0] << " " << x[1] << " " << x[2] << "]";
}

pcl::PolygonMesh computeCameraMesh(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist, float r, float g, float b, double s)
{
  const Eigen::Affine3d transformation(
    Eigen::Translation<double,3>(position[0],position[1], position[2]) *
    Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z())
  );

  const Eigen::Matrix3d& R = transformation.inverse().rotation();
  const Eigen::Vector3d& t = transformation.translation();

  // TODO: should be a parameter, however is does not seem to work.
  float alpha = 0.1;

  // create five points of the pyramid (camera view)
  pcl::PointCloud<pcl::PointXYZRGBA> mesh_cld;

  // define coordinate system
  Eigen::Vector3d vright = R.row(0).normalized() * s;
  Eigen::Vector3d vup = -R.row(1).normalized() * s;
  Eigen::Vector3d vforward = R.row(2).normalized() * s;

  mesh_cld.push_back(eigen2pcl(t, r, g, b, alpha));
  mesh_cld.push_back(eigen2pcl(t + vforward + vright + vup/2.0, r, g, b, alpha));
  mesh_cld.push_back(eigen2pcl(t + vforward + vright - vup/2.0, r, g, b, alpha));
  mesh_cld.push_back(eigen2pcl(t + vforward - vright + vup/2.0, r, g, b, alpha));
  mesh_cld.push_back(eigen2pcl(t + vforward - vright - vup/2.0, r, g, b, alpha));

  //~ FrustumCulling frustum(position, orientation, horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);
  //~ Eigen::Vector3d bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner;
  //~ frustum.GetFarPlaneCorners(bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner);

  //~ mesh_cld.push_back(eigen2pcl(t + topRightCorner, r, g, b, alpha));
  //~ mesh_cld.push_back(eigen2pcl(t + bottomRightCorner/2.0, r, g, b, alpha));
  //~ mesh_cld.push_back(eigen2pcl(t + topLeftCorner/2.0, r, g, b, alpha));
  //~ mesh_cld.push_back(eigen2pcl(t + bottomLeftCorner/2.0, r, g, b, alpha));

  // create polygon mesh
  pcl::PolygonMesh pm;
  pm.polygons.resize(4);

  // set the triangles of the pyramid with the defined vertices
  std::vector<unsigned int> vertices_triangle_left = {0, 1, 2};
  std::vector<unsigned int> vertices_triangle_right = {0, 3, 4};
  std::vector<unsigned int> vertices_triangle_up = {0, 1, 3};
  std::vector<unsigned int> vertices_triangle_down = {0, 2, 4};

  pm.polygons[0].vertices = vertices_triangle_left;
  pm.polygons[1].vertices = vertices_triangle_right;
  pm.polygons[2].vertices = vertices_triangle_up;
  pm.polygons[3].vertices = vertices_triangle_down;

  // convert pcl::PointCloud to pcl::PointCloud2 and save it
  pcl::toPCLPointCloud2(mesh_cld, pm.cloud);

  return pm;
}

void pointPickingActionCallBack(const pcl::visualization::PointPickingEvent& event)
{
  if (event.getPointIndex () == -1)
    return;

  pcl::PointXYZRGBA current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}
