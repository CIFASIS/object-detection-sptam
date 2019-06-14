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
 *           Matias Nitschẹ <mnitsche at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */


#include "TrackingReport.hpp"
#include "utils/draw/Draw.hpp"

TrackingReport::TrackingReport(const cv::Mat& left_in, const cv::Mat& right_in)
  : left_in_( left_in ), right_in_( right_in )
{
  state = State::OK;
  draw_output = (DrawOutput)(DRAW_AFTER_REFINE_STEREO | DRAW_BEFORE_REFINE_STEREO);
}

#define DRAW_COVARIANCES 0

void TrackingReport::drawStereoFrame(const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points,
                                     const std::list<Match>& measurements, const Parameters& params, bool before_refine)
{
#ifdef SHOW_TRACKED_FRAMES
  /* check if no output is expected */
  if (draw_output == DRAW_NONE)
    return;

  /* check if currently requested output is expected */
  if ((before_refine &&
       !(draw_output & DRAW_BEFORE_REFINE_LEFT) &&
       !(draw_output & DRAW_BEFORE_REFINE_RIGHT))
      ||
      (!before_refine &&
       !(draw_output & DRAW_AFTER_REFINE_LEFT) &&
       !(draw_output & DRAW_AFTER_REFINE_RIGHT)
      ))
    return;

  /* check if input images are valid */
  if (left_in_.empty() || right_in_.empty())
    return;    

  cv::Mat left_out, right_out;

  /* create stereo outputs if required */
  if (before_refine && (draw_output & DRAW_BEFORE_REFINE_STEREO))
    stereoFrameBeforeRefine = makeStereoWindow(left_in_, right_in_, left_out, right_out);
  else if (!before_refine && (draw_output & DRAW_AFTER_REFINE_STEREO))
    stereoFrameAfterRefine = makeStereoWindow(left_in_, right_in_, left_out, right_out);

  /* create individual outputs if required */
  if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    makeColorCopy(left_in_, left_out);
  if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    makeColorCopy(right_in_, right_out);

  /* map outputs */
  if (before_refine)
  {
    leftFrameBeforeRefine = left_out;
    rightFrameBeforeRefine = right_out;
  }
  else
  {
    leftFrameAfterRefine = left_out;
    rightFrameAfterRefine = right_out;
  }
  
  //Javier
  //
  //return ;

  if (before_refine)
  {
    //javier no grid
    //if (draw_output & DRAW_BEFORE_REFINE_LEFT) drawGrid(left_out, params.matchingCellSize);
    //if (draw_output & DRAW_BEFORE_REFINE_RIGHT) drawGrid(right_out, params.matchingCellSize);
  }

  if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    drawProjections(left_out, frame.GetFrameLeft().GetProjection(), filtered_points);
  if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    drawProjections(right_out, frame.GetFrameRight().GetProjection(), filtered_points );

#if DRAW_COVARIANCES
  if (before_refine)
  {
    if (draw_output & DRAW_BEFORE_REFINE_LEFT) drawProjectionCovariances(frame.GetFrameLeft(), left_out, filtered_points);
    if (draw_output & DRAW_BEFORE_REFINE_RIGHT) drawProjectionCovariances(frame.GetFrameRight(), right_out, filtered_points);
  }
#endif

  // extract and draw unmatched keypoints
  if (before_refine)
  {
    std::vector<cv::KeyPoint> keyPointsLeft, keyPointsRight;
    cv::Mat descriptorsLeft, descriptorsRight;
    std::vector<size_t> indexesLeft, indexesRight;

    if (draw_output & DRAW_BEFORE_REFINE_LEFT)
    {
      frame.GetFrameLeft().GetUnmatchedKeyPoints(keyPointsLeft, descriptorsLeft, indexesLeft);
      cv::drawKeypoints(left_out, keyPointsLeft, left_out, COLOR_CYAN);
    }

    if (draw_output & DRAW_BEFORE_REFINE_RIGHT)
    {
      frame.GetFrameRight().GetUnmatchedKeyPoints(keyPointsRight, descriptorsRight, indexesRight);
      cv::drawKeypoints(right_out, keyPointsRight, right_out, COLOR_CYAN);
    }
  }

  if ((draw_output & DRAW_BEFORE_REFINE_STEREO) || (draw_output & DRAW_AFTER_REFINE_STEREO))
    drawMeasuredFeatures(frame, measurements, left_out, right_out);
  else if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    drawMeasuredFeatures(frame, measurements, left_out, true);
  else if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    drawMeasuredFeatures(frame, measurements, right_out, false);
#endif
}

void TrackingReport::drawLeftFrame(const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points, const std::list<Match>& measurements, const Parameters& params, bool before_refine)
{
#ifdef SHOW_TRACKED_FRAMES
  
  //cv::Mat left_out(left_in_.rows, left_in_.cols, CV_8UC3);
  
  //if (before_refine) { 
  //  leftFrameBeforeRefine = left_out; 
  //} else {
  //  leftFrameAfterRefine = left_out;
  //} 

  //left_in_.copyTo( left_out );
 
  //drawProjections(left_out, frame.GetFrameLeft().GetProjection(), filtered_points );
  //drawProjectionCovariances(frame.GetFrameLeft(), outImageLeft, filtered_points);
  //drawProjectionCovariances(frame.GetFrameRight(), outImageRight, filtered_points);

  //drawMeasuredFeatures(frame, measurements, left_out, right_out);
#endif
}
//template void TrackingReport::drawLeftFrame<sptam::Map::SharedMapPointList>(const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points, const std::list<Match>& measurements, const Parameters& params, bool before_refine) ;
//template void TrackingReport::drawLeftFrame<sptam::ObjectMap::ObjectList>(const StereoFrame& frame, const sptam::ObjectMap::ObjectList& filtered_points, const std::list<Match>& measurements, const Parameters& params, bool before_refine) ;

template<class T>
void TrackingReport::drawPoints(const StereoFrame& frame, const T& points, bool before_refine)
{
#ifdef SHOW_TRACKED_FRAMES

  cv::Mat left_out,left_in ;


  if (before_refine) {
    left_in = leftFrameBeforeRefine ;
  } else {
    left_in = leftFrameAfterRefine ;
  }
  
  makeColorCopy(left_in, left_out) ; 

  if (before_refine) {
    leftFrameBeforeRefine = left_out ;
  } else {
    leftFrameAfterRefine = left_out ;
  }
  
  T filtered_points ;
  cv::Vec3d color ; 
 
  for(const auto& mapPoint : points ) {   
    const Eigen::Vector3d& point = mapPoint->GetPosition();
    //bool similar_angle = false;
    //Eigen::Vector3d currentNormal = point - frame.GetPosition();
    //currentNormal.normalize();
    // angle is in radians
    //double angle = std::acos( ( mapPoint->GetNormal() ).dot( currentNormal ) );
    // Discard Points which were created from a greater 45 degrees pint of view.
    // TODO: pass this threshold as a parameter.
    // similar_angle = angle < (M_PI / 4.0);
     if (frame.GetFrameLeft().GetCamera().CanView( point )) {
        color = mapPoint->getColor() ;
        filtered_points.push_back(mapPoint) ;
     }
   }
 
   drawProjections(left_out, frame.GetFrameLeft().GetProjection(), filtered_points,color );
  //~ drawProjectionCovariances(frame.GetFrameLeft(), outImageLeft, filtered_points);
  //~ drawProjectionCovariances(frame.GetFrameRight(), outImageRight, filtered_points);

  //drawMeasuredFeatures(frame, measurements, left_out, right_out);
#endif
}

template void TrackingReport::drawPoints<sptam::Map::SharedMapPointList>(const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points, bool before_refine) ;
template void TrackingReport::drawPoints<sptam::ObjectMap::ObjectList>(const StereoFrame& frame, const sptam::ObjectMap::ObjectList& filtered_points, bool before_refine) ;
template void TrackingReport::drawPoints<ObjectPointList>(const StereoFrame& frame, const ObjectPointList& filtered_points, bool before_refine) ;


void TrackingReport::drawObjects(const StereoFrame& frame, const sptam::ObjectMap::ObjectList& objects, bool before_refine) {
#ifdef SHOW_TRACKED_FRAMES

  cv::Mat left_out,left_in ;


  if (before_refine) {
    left_in = leftFrameBeforeRefine ;
  } else {
    left_in = leftFrameAfterRefine ;
  }

  makeColorCopy(left_in, left_out) ;

  if (before_refine) {
    leftFrameBeforeRefine = left_out ;
  } else {
    leftFrameAfterRefine = left_out ;
  }

  for(const auto& object : objects ) {
    const Eigen::Vector3d& pos = object->GetPosition();
    const Eigen::Matrix3d& ori = object->GetOrientationMatrix() ;
    const Eigen::Vector3d& D = object->GetDimensions() ;

    Eigen::Vector3d boxPoint[] = { Eigen::Vector3d( D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                   Eigen::Vector3d(-D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                   Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                   Eigen::Vector3d( D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                   Eigen::Vector3d( D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                   Eigen::Vector3d(-D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                   Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0,-D(2)/2.0) ,
                                   Eigen::Vector3d( D(0)/2.0,-D(1)/2.0,-D(2)/2.0) } ;

    int canView = 0 ;
    for(int i = 0 ; i < 8 ; i++) 
      if (frame.GetFrameLeft().GetCamera().CanView(ori*boxPoint[i]+pos)) {
        canView++ ;
      }     

   
    //canView = frame.GetFrameLeft().GetCamera().CanView(pos);

    if (canView > 3) {
      
       ObjectPointList points ;
       /*
       Eigen::Vector3d boxPoint[] = { Eigen::Vector3d( D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                       Eigen::Vector3d(-D(0)/2.0, D(1)/2.0, D(2)/2.0) ,
                                       Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                       Eigen::Vector3d( D(0)/2.0,-D(1)/2.0, D(2)/2.0) ,
                                       Eigen::Vector3d( D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                       Eigen::Vector3d(-D(0)/2.0, D(1)/2.0,-D(2)/2.0) ,
                                       Eigen::Vector3d(-D(0)/2.0,-D(1)/2.0,-D(2)/2.0) ,
                                       Eigen::Vector3d( D(0)/2.0,-D(1)/2.0,-D(2)/2.0) } ;

       */
       for(int i = 0 ; i < 8 ; i++) {
         SharedObjectPoint a (new ObjectPoint(ori*boxPoint[i]+pos)) ; 
         points.push_back(a) ;
       }

       drawOBB(left_out, frame.GetFrameLeft().GetProjection(), points, object->GetClassName());
       drawProjections(left_out, frame.GetFrameLeft().GetProjection(), object->GetObjectDepthPoints());

         
 
    }
  }




#endif
} 



void TrackingReport::enableDrawOutput(TrackingReport::DrawOutput output)
{
  draw_output = (DrawOutput)(draw_output | output);
}

