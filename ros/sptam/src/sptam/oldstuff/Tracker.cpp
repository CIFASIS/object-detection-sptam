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

#include "Tracker.hpp"

#include "WLS.hpp"
#include "MEstimator.hpp"
#include "FrustumCulling.hpp"
#include "CameraParameters.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"
#include "camera_derivatives.hpp"

#ifdef SHOW_PROFILING
  #include "utils/Profiler.hpp"
#endif // SHOW_PROFILING

#include <opencv2/calib3d/calib3d.hpp>

// constructor of class Tracker
Tracker::Tracker(const cv::Matx33d& intrinsicLeft, const cv::Matx33d& intrinsicRight, double stereo_baseline)
  : intrinsicLeft_( intrinsicLeft ), intrinsicRight_( intrinsicLeft ), stereo_baseline_( stereo_baseline )
{}

CameraPose Tracker::RefineCameraPose(
  const CameraPose& estimatedCameraPose,
  const std::vector<Measurement>& measurementsLeft,
  const std::vector<Measurement>& measurementsRight
)
{
  size_t numMeas = measurementsLeft.size() + measurementsRight.size();

  // We need at least 4 measurements to get a solution.
  // Make sure there are a bunch more just to be robust.
  if( numMeas < 10 ) {
    std::cerr << std::endl << std::endl << "WARNING: Not enough points for tracking." << std::endl << std::endl;
    return estimatedCameraPose;
  }

  // Set initial estimate of the camera pose to be adjusted
  cv::Matx44d updatedCameraPose = estimatedCameraPose.GetSE3Matrix();
  cv::Matx44d previousCameraPose = updatedCameraPose;

  size_t maxIterationNumber = 10; // TODO: pasar como parametro

  // Measured squared error for each reprojection on previous iteration.
  std::vector<double> errorSquaredsLast( numMeas );

  // Measured variance for previous iteration.
  // Used to check if error decreased over the last step.
  // Initialize at highest possible value.
  double dSigmaSquaredLast = std::numeric_limits<double>::max();

  // make ten gauss-newton pose update iterations
  for(size_t iter = 0; iter < maxIterationNumber; ++iter) {

    // declare vector variables and reserve space for them

    cv::Matx26d jacobians[ numMeas ];
    cv::Point2d error_CovScaleds[ numMeas ];
    // Measured squared error for each reprojection on current iteration.
    std::vector<double> errorSquareds( numMeas );
    // Measured variance for current iteration.
    double dSigmaSquared;

    // Find the covariance-scaled reprojection error for each measurement.
    // Also, store the square of these quantities for M-Estimator sigma squared estimation.
    size_t i = 0;

    for( const auto& meas : measurementsLeft ) {

      cv::Point3d point = meas.mapPoint->GetPosition();

      // compute jacobian

      cv::Point3d pointInPreviousIterationFrame = transform( previousCameraPose, point );
      cv::Point3d pointInRefinedCameraFrame = transform( updatedCameraPose, point );

      jacobians[ i ] = ComputeJacobian(intrinsicLeft_(0, 0), intrinsicLeft_(1, 1), pointInPreviousIterationFrame, pointInRefinedCameraFrame);

      // project the point to image plane

      cv::Point2d pointProjection = toInHomo( intrinsicLeft_ * pointInRefinedCameraFrame );
      cv::Point2d measurement = meas.GetProjection();

      // Compute the measurement error

      error_CovScaleds[ i ] = measurement - pointProjection;
      errorSquareds[ i ] = error_CovScaleds[ i ].dot( error_CovScaleds[ i ] );

      i++;
    }

    for( const auto& meas : measurementsRight ) {

      cv::Point3d point = meas.mapPoint->GetPosition();

      // compute jacobian

      cv::Point3d pointInPreviousIterationFrame = transform( previousCameraPose, point ) - cv::Point3d(stereo_baseline_, 0, 0);
      cv::Point3d pointInRefinedCameraFrame = transform( updatedCameraPose, point ) - cv::Point3d(stereo_baseline_, 0, 0);

      jacobians[ i ] = ComputeJacobian(intrinsicRight_(0, 0), intrinsicRight_(1, 1), pointInPreviousIterationFrame, pointInRefinedCameraFrame);

      // project the point to image plane

      cv::Point2d pointProjection = toInHomo( intrinsicRight_ * pointInRefinedCameraFrame );
      cv::Point2d measurement = meas.GetProjection();

      // Compute the measurement error

      error_CovScaleds[ i ] = measurement - pointProjection;
      errorSquareds[ i ] = error_CovScaleds[ i ].dot( error_CovScaleds[ i ] );

      i++;
    }

    // Compute the error distribution
    dSigmaSquared = Tukey::FindSigmaSquared( errorSquareds );

    //std::cout << " > GN dSigmaSquared: " << dSigmaSquared << std::endl;

    // if the sigmaSquare does not get (much) better
    // make the previous iteration the last one.
    //if ( (dSigmaSquaredLast - dSigmaSquared) < deltaSigmaSquared ) {
    if ( dSigmaSquaredLast <= dSigmaSquared ) {
      std::cout << "break tracking at iter number: " << iter << std::endl;
      break;
    }

    // Weight Least Squared Class
    WLS wls;

    // sirve para que el mu devuelto sea un movimiento corto.
    // Stabilising prior (esta en la thesis es la P, pagina 63 ecuacion 4.15)
    wls.add_prior(100.0);

    forn(i, numMeas) {

      double dWeight = Tukey::Weight(errorSquareds[i], dSigmaSquared);

      // se puede calcular directamente cuando se hace el matching
      const cv::Matx26d& m26Jac = jacobians[i];

      const cv::Point2d& v2 = error_CovScaleds[i];

      wls.Add_mJ( v2.x, cv::Vec6d(cv::Mat( m26Jac.row(0) )), dWeight );
      wls.Add_mJ( v2.y, cv::Vec6d(cv::Mat( m26Jac.row(1) )), dWeight );

      i++;
    }

    // update error measurements
    dSigmaSquaredLast = dSigmaSquared;
    errorSquaredsLast = errorSquareds;

    // update de motion vector
    wls.Compute();
    cv::Vec6d motion = wls.GetMu();

    // convert motion to SE(3) Matrix and
    // update the refined camera pose.
    previousCameraPose = updatedCameraPose;
    updatedCameraPose = ExponentialMap( motion ) * updatedCameraPose;
  }

  // Check if result is good by some heuristic
  // TODO hacerlo bien!!! parametrizar?

  //if ( 5000 < dSigmaSquaredLast )
    //return estimatedCameraPose;

  // Inlier / outlier accounting.
  // Only really works for cut-off estimators such as Tukey.

  // Number of outliers of last iteration.
  size_t nOutliers = 0;

  #ifdef SHOW_PROFILING
    std::list<double> inlier_errors_squared, outlier_errors_squared;
  #endif

  {
    size_t i=0;

    for( auto& meas : measurementsLeft ) {

      double dWeight = Tukey::Weight(errorSquaredsLast[ i ], dSigmaSquaredLast);

      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      MapPoint* mapPoint = meas.mapPoint;

      if( dWeight == 0.0 ) {
        mapPoint->IncreaseOutlierCount();
        #ifdef SHOW_PROFILING
          outlier_errors_squared.push_back( errorSquaredsLast[ i ] );
        #endif
        nOutliers++;
      } else {
        mapPoint->IncreaseInlierCount();
        #ifdef SHOW_PROFILING
          inlier_errors_squared.push_back( errorSquaredsLast[ i ] );
        #endif
      }

      i++;
    }

    for( auto& meas : measurementsRight ) {

      double dWeight = Tukey::Weight(errorSquaredsLast[ i ], dSigmaSquaredLast);

      MapPoint* mapPoint = meas.mapPoint;

      if( dWeight == 0.0 ) {
        mapPoint->IncreaseOutlierCount();
        #ifdef SHOW_PROFILING
          outlier_errors_squared.push_back( errorSquaredsLast[ i ] );
        #endif
        nOutliers++;
      } else {
        mapPoint->IncreaseInlierCount();
        #ifdef SHOW_PROFILING
          inlier_errors_squared.push_back( errorSquaredsLast[ i ] );
        #endif
      }

      i++;
    }
  }

//  #ifdef SHOW_PROFILING
//  WriteToLog( " tk sigma_squared: ", dSigmaSquaredLast );
//  WriteToLog( " tk error_squared_inliers: ", inlier_errors_squared );
//  WriteToLog( " tk error_squared_outliers: ", outlier_errors_squared );
//  WriteToLog( " tk outliers: ", nOutliers );
//  #endif

  cv::Matx33d refinedRotation;
  cv::Vec3d refinedTranslation;
  DecomposeTransformation44(updatedCameraPose, refinedRotation, refinedTranslation);

  return CameraPose(refinedTranslation, refinedRotation);
}

cv::Matx26d ComputeJacobian(double fu, double fv, const cv::Point3d& pointInPreviousCameraFrame, const cv::Point3d& pointInRefinedCameraFrame)
{
    // Compute point derivative over the camera parameters

  cv::Point3d dX[ 6 ];
  dX[ 0 ] = cv::Point3d(1, 0, 0);
  dX[ 1 ] = cv::Point3d(0, 1, 0);
  dX[ 2 ] = cv::Point3d(0, 0, 1);
  dX[ 3 ] = cv::Point3d(0, -pointInPreviousCameraFrame.z, pointInPreviousCameraFrame.y);
  dX[ 4 ] = cv::Point3d(pointInPreviousCameraFrame.z, 0, -pointInPreviousCameraFrame.x);
  dX[ 5 ] = cv::Point3d(-pointInPreviousCameraFrame.y, pointInPreviousCameraFrame.x, 0);

    // Compute point derivative over the camera parameters

  // variables for legibility
  double oneOverPointZc = 1.0 / pointInRefinedCameraFrame.z;

  // compute the jacobian
  cv::Matx26d m26Jacobian;

  for(int j=0; j<6; j++) {

    m26Jacobian(0, j) = fu * (dX[j].x - pointInRefinedCameraFrame.x * dX[j].z * oneOverPointZc) * oneOverPointZc;
    m26Jacobian(1, j) = fv * (dX[j].y - pointInRefinedCameraFrame.y * dX[j].z * oneOverPointZc) * oneOverPointZc;
  }

  return m26Jacobian;
}

cv::Matx44d ExponentialMap(const cv::Vec6d& mu)
{
  const cv::Vec3d translation(mu[0], mu[1], mu[2]);

  const cv::Vec3d w(mu[3], mu[4], mu[5]);

  cv::Matx33d rotation;
  cv::Rodrigues(w, rotation);

  return ComputeTransformation44(rotation, translation);
}
