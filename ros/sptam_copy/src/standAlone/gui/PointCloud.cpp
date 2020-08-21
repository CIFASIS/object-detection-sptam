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

#include "PointCloud.hpp"

#ifdef SHOW_PROFILING
  #include "../../sptam/utils/Profiler.hpp"
#endif // SHOW_PROFILING

static Eigen::Vector3d cameraPointCloudLastPos;


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// CALLBACKS stuffs for visualization

// Define some Types for easier writing
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Callback function called by registerPointPickingCallback() PCL function.
// Use Shift+Click to select a point in the Viewer
void PointPickingActionCallBack (const pcl::visualization::PointPickingEvent& event)
{
  if (event.getPointIndex () == -1)
    return;

  pcl::PointXYZRGBA current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Clase que corre el thread del PointCloud
PointCloud::PointCloud(CameraPose& cameraPoseTracked, const sptam::Map& map,
                       const double horizontalFOV, const double verticalFOV,
                       const double nearPlaneDist, const double farPlaneDist)
  : cameraPoseTracked_(cameraPoseTracked), map_(map)
{
  // Set Frustum Parameters
  horizontalFOV_ = horizontalFOV;
  verticalFOV_ = verticalFOV;
  nearPlaneDist_ = nearPlaneDist;
  farPlaneDist_ = farPlaneDist;

  stop_ = false;
  updated = true;
  
  thread_ = std::thread(&PointCloud::operator(), this);
}

// ------------------------------------------------------------------------------------------------------------

void PointCloud::operator()(){


  if (viewer_ == NULL) {
    //Start visualizer thread
    viewer_ = create3DMapVisualizer(map_, horizontalFOV_, verticalFOV_, nearPlaneDist_, farPlaneDist_);
  }

  while (!stop_ || viewer_->wasStopped()) {

    // Get lock on the boolean update and check if cloud was updated
    boost::mutex::scoped_lock updateLock( updateModelMutex );
    viewer_->spinOnce(100);

    if( updated ) {
      // Compute Frustum Culling using the FOV
      FrustumCulling frustumCulling(cameraPoseTracked_.GetPosition(), cameraPoseTracked_.GetOrientationQuaternion(), horizontalFOV_,  verticalFOV_, nearPlaneDist_, farPlaneDist_);
      draw3DMapAndCurrentCameraPose(cameraPoseTracked_, map_, viewer_, frustumCulling);
      updated = false;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------

void PointCloud::Stop()
{
  stop_ = true;
  thread_.join();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////


pcl::PointXYZRGBA Eigen2PointXYZRGBA(Eigen::Vector3f v, Eigen::Vector4f rgba) {
  pcl::PointXYZRGBA p;
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
  p.r = rgba[0];
  p.g = rgba[1];
  p.b = rgba[2];
  p.a = rgba[3];
  return p;
}

pcl::PolygonMesh visualizerGetCameraMesh(const CameraPose& cameraPose, float r, float g, float b, double s = 1.0)
{

  const Eigen::Vector3d& position = cameraPose.GetPosition();
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();
  const Eigen::Affine3f transformation(Eigen::Translation<float,3>(position[0],position[1], position[2]) *
                                 Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z()));


  const Eigen::Matrix3f& R = transformation.inverse().rotation();
  const Eigen::Vector3f& t = transformation.translation();

  // define coordinate system
  Eigen::Vector3f vright = R.row(0).normalized() * s;
  Eigen::Vector3f vup = -R.row(1).normalized() * s;
  Eigen::Vector3f vforward = R.row(2).normalized() * s;

  float alpha = 150; // TODO: it must be a parameter, however is not working the opacity.
  Eigen::Vector4f rgba = Eigen::Vector4f(r,g,b, alpha);

  // create five points of the pyramid (camera view)
  pcl::PointCloud<pcl::PointXYZRGBA> mesh_cld;
  mesh_cld.push_back(Eigen2PointXYZRGBA(t,rgba));
  mesh_cld.push_back(Eigen2PointXYZRGBA(t + vforward + vright + vup/2.0,rgba));
  mesh_cld.push_back(Eigen2PointXYZRGBA(t + vforward + vright - vup/2.0,rgba));
  mesh_cld.push_back(Eigen2PointXYZRGBA(t + vforward - vright + vup/2.0,rgba));
  mesh_cld.push_back(Eigen2PointXYZRGBA(t + vforward - vright - vup/2.0,rgba));

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

// ------------------------------------------------------------------------------------------------------------

template<class GenericContainer>
pcl::PolygonMesh visualizerConcatenatePolygonmeshes(GenericContainer& meshes) {
  pcl::PolygonMesh output;
  typename GenericContainer::iterator current = meshes.begin();
  typename GenericContainer::iterator end = meshes.end();

  pcl::PointCloud<pcl::PointXYZRGBA> allpoints;
  while (current != end)
  {
    int last_poly_idx = allpoints.size();

    pcl::PolygonMesh& current_poly = *current;
    pcl::PointCloud<pcl::PointXYZRGBA> tmppoints;
    pcl::fromPCLPointCloud2(current_poly.cloud, tmppoints);
    allpoints.insert(allpoints.end(),tmppoints.begin(),tmppoints.end()); //add points

    for(uint i=0;i<current_poly.polygons.size();i++) {
      pcl::Vertices vertices = current_poly.polygons[i];
      for(uint j=0;j<vertices.vertices.size();j++)
        vertices.vertices[j] += last_poly_idx;
      output.polygons.push_back(vertices);
    }

    current++;
  }
  pcl::toPCLPointCloud2(allpoints,output.cloud);
  return output;
}

// ------------------------------------------------------------------------------------------------------------

// Crea una point cloud colorea de rojo a los puntos que estan dentro del Frustum
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr createPointCloud(const sptam::Map& map)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA> );

  {
  // Get the lock
  boost::unique_lock<boost::shared_mutex> lock(map.map_mutex_);

  #ifdef SHOW_PROFILING
  double start = GetSeg();
  #endif

  for ( const auto& mapPoint : map.getMapPoints() ) {

    const Eigen::Vector3d& point3d = mapPoint->GetPosition();

    // show only point which distance z is less than 600mm.
    // There are points that are far (60cm) from the camera that are
    // not visualized but are still used for SPTAM processing
    // this happens because the visualizer scales the cloud to contain
    // all the points and it is not possible to appreciate the cloud properly
    if ( /*600 < std::abs( point.x ) or 600 < std::abs( point.y ) or*/ 600 < std::abs( point3d[2] ) )
			continue;

    // create PCL point
    pcl::PointXYZRGBA point_pcl;
    point_pcl.x = point3d[0];
    point_pcl.y = point3d[1];
    point_pcl.z = point3d[2];

    // show bad points in red, otherwise real color
    if ( mapPoint->IsBad() ) {
      point_pcl.r = 255;
      point_pcl.g = 0;
      point_pcl.b = 0;
    }
    else {
      cv::Vec3b color = mapPoint->getColor();

      point_pcl.r = color(0);
      point_pcl.g = color(1);
      point_pcl.b = color(2);
    }

    // Add point to the pointCloud
    pointCloudPtr->points.push_back ( point_pcl );
  }

  #ifdef SHOW_PROFILING
  double end = GetSeg();
  WriteToLog(" pcl points_visualization: ", start, end);
  #endif

  }


  pointCloudPtr->width = (int) pointCloudPtr->points.size();
  pointCloudPtr->height = 1;
  return pointCloudPtr;
 }


// ------------------------------------------------------------------------------------------------------------

void addCameraPose(const CameraPose& cameraPose, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  // Eigen::Affine3f transformation(Eigen::Translation<float,3>(state.position[0], state.position[1], state.position[2]) *
  //                                Eigen::Quaternionf(state.orientation[0], state.orientation[1], state.orientation[2], state.orientation[3]));

  // Eigen::Affine3f rotationMatrix;
  // rotationMatrix =
  //   Eigen::AngleAxisf(state.orientationEuler[0], Eigen::Vector3f::UnitX())
  //   * Eigen::AngleAxisf(state.orientationEuler[1], Eigen::Vector3f::UnitY())
  //   * Eigen::AngleAxisf(state.orientationEuler[2], Eigen::Vector3f::UnitZ());

  // Eigen::Affine3f transformation(Eigen::Translation<float,3>(state.position[0], state.position[1], state.position[2]) * rotationMatrix);


  const Eigen::Vector3d& position = cameraPose.GetPosition();
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();
  Eigen::Affine3f transformation( Eigen::Translation<float,3>(position[0],position[1], position[2]) *
                                  Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z()) );

  viewer->addCoordinateSystem(0.5f, transformation);


  // Show Camera trajectory
  std::ostringstream s;
  s << "path" << "(" << position[0] << ", " << position[1] << ", " << position[2] << ")";
  ShowLine(cameraPointCloudLastPos,position, 255.0, 255.0, 0, viewer, s.str());
  cameraPointCloudLastPos[0] = position[0];
  cameraPointCloudLastPos[1] = position[1];
  cameraPointCloudLastPos[2] = position[2];
}

// ------------------------------------------------------------------------------------------------------------

void addCameraPoses(const std::vector<StereoFrame*>& keyFrames, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  for(uint i = 0 ; i < keyFrames.size(); ++i) {
    const CameraPose& cameraPose = keyFrames[i]->GetCameraPose();
    addCameraPose(cameraPose, viewer);
  }
}

// ------------------------------------------------------------------------------------------------------------

boost::shared_ptr<pcl::visualization::PCLVisualizer> create3DMapVisualizer(const sptam::Map &map,
                                                                           const double horizontalFOV = 0,
                                                                           const double verticalFOV = 0,
                                                                           const double nearPlaneDist = 0,
                                                                           const double farPlaneDist = 0)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3D Viewer") );

  // Add point cloud
  // dummy variable, just used to pass as argument initially to createPointCloud
  FrustumCulling frustumCulling(Eigen::Vector3d(0,0,0), Eigen::Quaterniond::Identity(), horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr = createPointCloud(map);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pointCloudPtr);

  viewer->addPointCloud<pcl::PointXYZRGBA>(pointCloudPtr, rgb, "map");

  // Init other parameters
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.5f);

  // Init Camera Last Pose (for show camera trajectory)
  cameraPointCloudLastPos[0] = 0;
  cameraPointCloudLastPos[1] = 0;
  cameraPointCloudLastPos[2] = 0;

  //valores del viewer para apreciar mejor la camara
//  viewer->setCameraPosition(0, 0, -2, 0, -1, 0);
  viewer->setCameraPosition(-24.5519,-69.6109,394.59,-780.751,-19.3139,397.785,-0.0660845,-0.996637,0.0484596);
  viewer->setCameraFieldOfView(0.8575);
  viewer->setCameraClipDistances(5.17196,5171.96);
  viewer->setPosition(425,137);
  viewer->setSize(683,384);

  // register point clicking (use PointPickingActionCallBack function callback)
  viewer->registerPointPickingCallback(PointPickingActionCallBack);

  return viewer;
}

// ------------------------------------------------------------------------------------------------------------

void draw3DMapAndCurrentCameraPose(const CameraPose& cameraPoseTracked,
                                   const sptam::Map& map,
                                   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                   FrustumCulling& frustumCulling)
{
  // Create Point Cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr = createPointCloud(map);

  viewer->removeCoordinateSystem();

  // Update Viewer with the new Point Cloud
  viewer->updatePointCloud(pointCloudPtr, "map");


  // Add the KeyFrames to the viewer
  std::list<pcl::PolygonMesh> keyframes_meshes;

  pcl::PolygonMesh keyframes_mesh;

  for(const auto& k : map.getKeyframes()) {

    pcl::PolygonMesh keyframe_mesh = visualizerGetCameraMesh(k->GetCameraPose(), 255, 0, 0);

    keyframes_meshes.push_back( keyframe_mesh );

    keyframes_mesh = visualizerConcatenatePolygonmeshes( keyframes_meshes );
  }

  std::string keyframes_mesh_id = "keyframes";

  viewer->removePolygonMesh( keyframes_mesh_id );

  viewer->addPolygonMesh(keyframes_mesh, keyframes_mesh_id);

  // Add current camera pose
  addCameraPose(cameraPoseTracked,viewer);

  // remove the previous Frustum
  viewer->removeShape("bottomLeftLine");
  viewer->removeShape("bottomRightLine");
  viewer->removeShape("topLeftLine");
  viewer->removeShape("topRightLine");
  viewer->removeShape("bottomLine");
  viewer->removeShape("topLine");
  viewer->removeShape("leftLine");
  viewer->removeShape("rightLine");

  // Draw Frustum
  Eigen::Vector3d bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner;
  frustumCulling.GetFarPlaneCorners(bottomLeftCorner, bottomRightCorner, topLeftCorner, topRightCorner);
  ShowLine(cameraPoseTracked.GetPosition(), bottomLeftCorner, 0, 0, 1.0f, viewer, "bottomLeftLine");
  ShowLine(cameraPoseTracked.GetPosition(), bottomRightCorner, 0, 0, 1.0f, viewer, "bottomRightLine");
  ShowLine(cameraPoseTracked.GetPosition(), topLeftCorner, 0, 0, 1.0f, viewer, "topLeftLine");
  ShowLine(cameraPoseTracked.GetPosition(), topRightCorner, 0, 0, 1.0f, viewer, "topRightLine");
  ShowLine(bottomLeftCorner, bottomRightCorner, 0, 0, 1.0f, viewer, "bottomLine");
  ShowLine(topLeftCorner, topRightCorner, 0, 0, 1.0f, viewer, "topLine");
  ShowLine(bottomLeftCorner, topLeftCorner, 0, 0, 1.0f, viewer, "leftLine");
  ShowLine(bottomRightCorner, topRightCorner, 0, 0, 1.0f, viewer, "rightLine");
}

// ------------------------------------------------------------------------------------------------------------
// ShowLine dibuja una linea entre dos puntos en el visualizador (es utilizada para dibujar el Frustum y la trayectoria de la camara)
void ShowLine(const Eigen::Vector3d& point1,
              const Eigen::Vector3d& point2,
              const double& red,
              const double& green,
              const double& blue,
              boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
              const std::string &id)
{
  pcl::PointXYZ p1, p2;

  p1.x = static_cast<float>(point1[0]);
  p1.y = static_cast<float>(point1[1]);
  p1.z = static_cast<float>(point1[2]);

  p2.x = static_cast<float>(point2[0]);
  p2.y = static_cast<float>(point2[1]);
  p2.z = static_cast<float>(point2[2]);

//   std::ostringstream s;
//   s << "path" << "(" << p2.x << ", " << p2.y << ", " << p2.z << ")";

  viewer->addLine(p1, p2, red, green, blue, id);
}

// ------------------------------------------------------------------------------------------------------------
// DrawPlane dibuja un plano en el visualizador con primitivas de PCL (esta funcion sirve para debugging)
void DrawPlane(const cv::Vec4f& plane, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, const std::string &id)
{
  pcl::ModelCoefficients plane_coeff;
  plane_coeff.values.resize (4);    // We need 4 values
  plane_coeff.values[0] = plane(0);
  plane_coeff.values[1] = plane(1);
  plane_coeff.values[2] = plane(2);
  plane_coeff.values[3] = plane(3);
  viewer->addPlane(plane_coeff, id);
}

// RemoveCameraPoses Remueve todas las Poses de la camaras en el visualizador
void RemoveCameraPoses(const std::vector<StereoFrame*>& keyFrames, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  for(uint i = 0 ; i < keyFrames.size(); ++i) {
    viewer->removeCoordinateSystem();
  }
}

void CreatePLYFile(const sptam::Map& map)
{
  ofstream out("pointCloud.ply");

  out << "ply" << endl;
  out << "format ascii 1.0" << endl;
  out << "element face 0" << endl;
  out << "property list uchar int vertex_indices" << endl;
  std::vector<Eigen::Vector3d> mapToShow;
  for (const auto& mapPoint : map.getMapPoints() ) {

    const Eigen::Vector3d& point = mapPoint->GetPosition();

    if (point[2] < 600)
      mapToShow.push_back(point);
  }
  out << "element vertex ";
  out << mapToShow.size() << endl;

  out << "property float x" << endl;
  out << "property float y" << endl;
  out << "property float z" << endl;
  out << "property uchar diffuse_red" <<endl;
  out << "property uchar diffuse_green" << endl;
  out << "property uchar diffuse_blue" << endl;
  out << "end_header" << endl;

  for ( auto& point : mapToShow) {

    //~ int color1 = 0;
    //~ int color2 = 0;
    //~ int color3 = 0;

    out << point[0] << " " << point[1] <<  " " << point[2] << " " << 0 << " "<< 0 << " "<< 0 << endl;
  }

//  for (unsigned int i = 0; i<map.nMapPoints(); i++) {
//    MapPoint &mapPoint = map[i];
//    double val1 = mapPoint->positionXYZ[0];
//    double val2 = mapPoint->positionXYZ[1];
//    double val3 = mapPoint->positionXYZ[2];
//
//    int color1 = 0;
//    int color2 = 0;
//    int color3 = 0;
//
//    out << val1 << " " << val2 <<  " " << val3 << " " << color1 << " "<< color2 << " "<< color3 << endl;
//
////     out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3
////         << " "<< color1 << " "<< color2 << " "<< color3 << endl;
//  }
   out.close();
}


