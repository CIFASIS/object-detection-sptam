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

#include "SptamWrapper.hpp"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "../sptam/utils/macros.hpp"

#ifdef SHOW_PROFILING
#include "../sptam/utils/log/Profiler.hpp"
#include "../sptam/utils/log/ScopedProfiler.hpp"
#endif

SptamWrapper::SptamWrapper(const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline, const RowMatcher& rowMatcher,
  const Parameters& params,
  std::shared_ptr<PosePredictor> motionModel,
  const size_t imageBeginIndex
)
  : cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , params_( params )
  , motionModel_( motionModel )
  , isMapInitialized_( false )
  , sptam_(rowMatcher, params)
{
  // Set camerapose from the MotionModel
  Eigen::Vector3d currentCameraPosition;
  Eigen::Quaterniond currentCameraOrientation;
  Eigen::Matrix6d covariance;
  motionModel_->currentPose(currentCameraPosition, currentCameraOrientation, covariance);
  currentCameraPose_= CameraPose(currentCameraPosition, currentCameraOrientation, covariance);
}

#ifdef USE_LOOPCLOSURE
SptamWrapper::SptamWrapper(
  const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline,
  const RowMatcher& rowMatcher,
  const Parameters& params,
  std::shared_ptr<PosePredictor>& motionModel,
  const size_t imageBeginIndex,
  std::unique_ptr<LCDetector>& loop_detector
)
  : cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , params_( params )
  , motionModel_( motionModel )
  , isMapInitialized_( false )
  , sptam_(rowMatcher, params, motionModel_, loop_detector)
{

  // Set camerapose from the MotionModel
  Eigen::Vector3d currentCameraPosition;
  Eigen::Quaterniond currentCameraOrientation;
  Eigen::Matrix6d covariance;
  motionModel_->currentPose(currentCameraPosition, currentCameraOrientation, covariance);
  currentCameraPose_= CameraPose(currentCameraPosition, currentCameraOrientation, covariance);
}
#endif

void SptamWrapper::Add(const size_t frame_id, const ros::Time& time, std::unique_ptr<StereoImageFeatures> stereoImageFeatures)
{
  ImageFeatures& imageFeaturesLeft = stereoImageFeatures->imageFeaturesLeft;
  ImageFeatures& imageFeaturesRight = stereoImageFeatures->imageFeaturesRight;

  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" tk trackingWithoutExtraction: ");
  #endif

  Eigen::Vector3d estimatedCameraPosition;
  Eigen::Quaterniond estimatedCameraOrientation;
  Eigen::Matrix6d predictionCovariance;

  // Check is there is no initial map create it
  if (not isMapInitialized_)
  {
    // HACK: cuando se utiliza el Ground-Truth hace que avance la pose adecuadamente (en el caso de que la primera imagen no sirva para inicializar el mapa)
    // Cuando se utiliza motion model no hace nada
    if ( frame_id != 0 ) {
      motionModel_->predictPose( time, estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance );
      currentCameraPose_ = CameraPose(estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance);
    }

    StereoFrame frame(
      frame_id,
      currentCameraPose_, cameraParametersLeft_,
      stereo_baseline_, cameraParametersRight_,
      imageFeaturesLeft, imageFeaturesRight, true
    );

    // Initialize MapMaker
    try {
      sptam_.init( frame );
      isMapInitialized_ = true;
    }
    catch( std::runtime_error& err ) {
      std::cerr << err.what() << std::endl;
    }
  }
  // if there is an initial map then run SPTAM
  else
  {
    motionModel_->predictPose( time, estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance );
    const CameraPose estimatedCameraPose( estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance );

    StereoFrame frame(
      frame_id,
      estimatedCameraPose, cameraParametersLeft_,
      stereo_baseline_, cameraParametersRight_,
      imageFeaturesLeft, imageFeaturesRight
    );

    // Main process
    TrackingReport report(stereoImageFeatures->imageLeft, stereoImageFeatures->imageRight);

    sptam_.track(frame, report);

    currentCameraPose_ = report.refinedCameraPose;

    #ifdef SHOW_TRACKED_FRAMES
    cv::imshow("Before Refine", report.stereoFrameBeforeRefine);
    cv::imshow("After Refine", report.stereoFrameAfterRefine);
    cv::waitKey( 1 );
    #endif
  } // else

  #ifdef SHOW_PROFILING
    writePoseToLog("TRACKED_FRAME_POSE", frame_id, currentCameraPose_.GetPosition(), currentCameraPose_.GetOrientationMatrix(), currentCameraPose_.covariance());
  #endif

  // Update motion model
  motionModel_->updatePose(time, currentCameraPose_.GetPosition(), currentCameraPose_.GetOrientationQuaternion(), currentCameraPose_.covariance());

  // Read the new camera pose because the motion model may apply
  // some post-processing to it, for example when using a filter.
  {
    Eigen::Vector3d currentCameraPosition; Eigen::Quaterniond currentCameraOrientation; Eigen::Matrix6d poseCovariance;
    motionModel_->currentPose(currentCameraPosition, currentCameraOrientation, poseCovariance);
    currentCameraPose_ = CameraPose( currentCameraPosition, currentCameraOrientation, poseCovariance );
  }
}
