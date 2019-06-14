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

#include "Map.hpp"
#include "BundleDriver.hpp"

////////////////////////////////////////////////////////////////////////////////

class PointVertexData : public G2ODriver::VertexData
{
  public:

    PointVertexData(const sptam::Map::SharedPoint& point)
      : point_( point ) {}

    virtual void saveData(g2o::OptimizableGraph::Vertex& vertex) override
    { point_->updatePosition( G2ODriver::GetPoint( vertex ) ); }

  private:

    sptam::Map::SharedPoint point_;
};

class CameraVertexData : public G2ODriver::VertexData
{
  public:

    CameraVertexData(const sptam::Map::SharedKeyFrame& keyFrame)
      : keyFrame_( keyFrame ) {}

    // TODO compute the new covariance for each adjusted pose
    // or at least use the old one...
    virtual void saveData(g2o::OptimizableGraph::Vertex& vertex) override
    { keyFrame_->UpdateCameraPose( G2ODriver::GetPose( vertex, Eigen::Matrix6d::Identity() ) ); }

  private:

    sptam::Map::SharedKeyFrame keyFrame_;
};

////////////////////////////////////////////////////////////////////////////////

void BundleDriver::SetData(ConstIterable<sptam::Map::SharedKeyFrame>& adjustViews, ConstIterable<sptam::Map::SharedKeyFrame>& fixedViews)
{
  // clear all previous information
  Clear();

  std::map<sptam::Map::Point*, G2ODriver::Vertex*> point_to_vertex;
  std::map<sptam::Map::KeyFrame*, G2ODriver::Vertex*> view_to_vertex;

  // Add the non-fixed keyframes' poses to the bundle adjuster.
  for ( const sptam::Map::SharedKeyFrame& keyFrame : adjustViews )
  {
    G2ODriver::Vertex* camera_vertex = g2o_driver_.AddVertex( keyFrame->GetCameraPose(), keyFrame->getRectifiedCameraParameters(), false, new CameraVertexData( keyFrame ) );
    camera_vertices_.push_back( camera_vertex );
    view_to_vertex[ keyFrame.get() ] = camera_vertex;

    for ( sptam::Map::SharedMeas& meas : keyFrame->measurements() )
    {
      sptam::Map::SharedPoint mapPoint = meas->mapPoint();

      G2ODriver::Vertex* point_vertex;

      // check if the point already exists in the driver.
      // If so, retrieve it.
      try
      {
        point_vertex = point_to_vertex.at( mapPoint.get() );
      }
      // if not, create it.
      catch (const std::out_of_range& err)
      {
        point_vertex = g2o_driver_.AddVertex( *mapPoint, true, false, new PointVertexData( mapPoint ) );
        point_vertices_.push_back( point_vertex );

        point_to_vertex[ mapPoint.get() ] = point_vertex;
      }

      g2o_driver_.AddEdge(measurements_.size(), point_vertex, camera_vertex, *meas);
      measurements_.push_back( meas );
    }
  }

  // Add the fixed keyframes' poses to the bundle adjuster.
  for ( const sptam::Map::SharedKeyFrame& keyFrame : fixedViews )
  {
    G2ODriver::Vertex* camera_vertex = g2o_driver_.AddVertex( keyFrame->GetCameraPose(), keyFrame->getRectifiedCameraParameters(), true, new CameraVertexData( keyFrame ) );
    view_to_vertex[ keyFrame.get() ] = camera_vertex;

    for ( sptam::Map::SharedMeas& meas : keyFrame->measurements() )
    {
      sptam::Map::SharedPoint mapPoint = meas->mapPoint();

      // add edges only for previously inserted points.
      try
      {
        G2ODriver::Vertex* point_vertex = point_to_vertex.at( mapPoint.get() );

        g2o_driver_.AddEdge(measurements_.size(), point_vertex, camera_vertex, *meas);
        measurements_.push_back( meas );
      }
      catch (const std::out_of_range& err)
      {}
    }
  }
}

void BundleDriver::SavePoints()
{
  for ( auto vertex : point_vertices_ )
  {
    G2ODriver::VertexData* data = dynamic_cast<G2ODriver::VertexData*>( vertex->userData() );
    data->saveData( *vertex );
  }
}

void BundleDriver::SaveCameras()
{
  for ( auto vertex : camera_vertices_ )
  {
    G2ODriver::VertexData* data = dynamic_cast<G2ODriver::VertexData*>( vertex->userData() );
    data->saveData( *vertex );
  }
}

std::list< sptam::Map::SharedMeas > BundleDriver::GetBadMeasurements()
{
  // threshold for a confidence interval of 95%
  const double huber_thresold = 5.991;

  std::list< sptam::Map::SharedMeas > badMeasurements;

  for ( auto& edge_ptr : g2o_driver_.activeEdges() )
    if ( huber_thresold < edge_ptr->chi2() )
      badMeasurements.push_back( measurements_[ edge_ptr->id() ] );

  return badMeasurements;
}

void BundleDriver::Clear()
{
  // freeing the graph memory
  g2o_driver_.Clear();

  point_vertices_.clear();
  camera_vertices_.clear();
  measurements_.clear();
}

void BundleDriver::Break()
{
  g2o_driver_.Break();
}

bool BundleDriver::Adjust(int maxIterations)
{
  return g2o_driver_.Adjust( maxIterations );
}
