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

#ifndef CAMERA_DERIVATIVES_HPP_
#define CAMERA_DERIVATIVES_HPP_

namespace cv {
  typedef Matx<double, 2, 6> Matx26d;
  typedef Matx<double, 2, 3> Matx23d;
  typedef Matx<double, 3, 6> Matx36d;
}

// Define generator matrices
/*const cv::Matx44d GENERATORS [6] =
{
  // Representa una traslacion sobre el eje x
  cv::Matx44d(
    0, 0, 0, 1,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
  ),

  // Representa una traslacion sobre el eje y
  cv::Matx44d(
    0, 0, 0, 0,
    0, 0, 0, 1,
    0, 0, 0, 0,
    0, 0, 0, 0
  ),

  // Representa una traslacion sobre el eje z
  cv::Matx44d(
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 1,
    0, 0, 0, 0
  ),

  // Representa una rotacion alrededor del eje x
  cv::Matx44d(
    0, 0, 0, 0,
    0, 0, -1, 0,
    0, 1, 0, 0,
    0, 0, 0, 0
  ),

  // Representa una rotacion alrededor del eje y
  cv::Matx44d(
    0, 0, 1, 0,
    0, 0, 0, 0,
    -1, 0, 0, 0,
    0, 0, 0, 0
  ),

  // Representa una rotacion alrededor del eje z
  cv::Matx44d(
    0, -1, 0, 0,
    1, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
  )
};*/

/**
 * Compute (D)proj(X_ij) / (D)X_ij where X_ij is the point X_i
 * in the coordinate frame of camera j.
 *
 * @param K
 *   Camera projection marix (3x3)
 * 
 * @param Xij
 *   point X_i in the coordinate frame of camera j.
 */
inline cv::Matx23d computeProyectionDerivative(double fu, double fv, const cv::Point3d& Xij)
{
  double zSqInv = 1.0 / ( Xij.z * Xij.z );

  return cv::Matx23d(
    fu / Xij.z, 0, - fu * Xij.x * zSqInv,
    0, fv / Xij.z, - fv * Xij.y * zSqInv
  );
}

/**
 * Compute (D)X_ij / (D)mu_j using the exponential map derivative,
 * where mu_j = (tx, ty, tz, roll, pitch, yaw)
 *
 * @param Xi
 *   point X_i in world frame coordinates
 */
inline cv::Matx36d computePointDerivative(const cv::Point3d& Xi)
{
  // el constructor no acepta todos estos parametros
  /*return cv::Matx36d(
    1, 0, 0, 0, Xi.z, -Xi.y,
    0, 1, 0, -Xi.z, 0, Xi.x,
    0, 0, 1, Xi.y, -Xi.x, 0
  );*/

  cv::Matx36d dX = cv::Matx36d::eye();

  // derivate respect to roll
  dX(0, 3) = 0;
  dX(1, 3) = -Xi.z;
  dX(2, 3) = Xi.y;

  // derivate respect to pitch
  dX(0, 4) = Xi.z;
  dX(1, 4) = 0;
  dX(2, 4) = -Xi.x;

  // derivate respect to yaw
  dX(0, 5) = -Xi.y;  
  dX(1, 5) = Xi.x;
  dX(2, 5) = 0;

  return dX;
}

inline cv::Matx26d getAij(double fu, double fv, const cv::Point3d& Xi, const cv::Point3d& Xij)
{
  cv::Matx23d dK = computeProyectionDerivative( fu, fv, Xij );
  cv::Matx36d dX = computePointDerivative( Xi );

  return dK * dX;
}

inline cv::Matx23d getBij(double fu, double fv, const cv::Matx33d& R, const cv::Point3d& Xij)
{
  cv::Matx23d dK = computeProyectionDerivative( fu, fv, Xij );

  return dK * R;
}

#endif // CAMERA_DERIVATIVES_HPP_
