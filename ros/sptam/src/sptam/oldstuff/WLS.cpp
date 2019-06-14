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

#include "WLS.hpp"

WLS::WLS()
{
  // motion que retorna
  my_mu = cv::Mat::zeros(6, 1, CV_64FC1);

  //
  my_C_inv = cv::Mat::zeros(6, 6, CV_64FC1);

  my_vector = cv::Mat::zeros(6, 1, CV_64FC1);

  my_decomposition = cv::Mat::zeros(6, 6 , CV_64FC1);
}

WLS::~WLS()
{
  //dtor
}

// PTAM original code
//00067     /// Applies a constant regularisation term.
//00068     /// Equates to a prior that says all the parameters are zero with \f$\sigma^2 = \frac{1}{\text{val}}\f$.
//00069     /// @param val The strength of the prior
//00070     void add_prior(Precision val){
//00071         for(int i=0; i<my_C_inv.num_rows(); i++){
//00072             my_C_inv(i,i)+=val;
//00073         }
//00074     }


void WLS::add_prior(double val)
{
  for(int i=0; i<my_C_inv.rows; i++){
    my_C_inv.at<double>(i,i)+=val;
  }
}

// PTAM original code
//00102         //Upper right triangle only, for speed
//00103         for(int r=0; r < my_C_inv.num_rows(); r++)
//00104         {
//00105             double Jw = weight * J[r];
//00106             my_vector[r] += m * Jw;
//00107             for(int c=r; c < my_C_inv.num_rows(); c++)
//00108                 my_C_inv[r][c] += Jw * J[c];
//00109         }

void WLS::Add_mJ(double m, const cv::Vec6d& Jrow, double weight)
{
  //Upper right triangle only, for speed
  for(int r=0; r < my_C_inv.rows; r++) {
    double Jw = weight * Jrow[r];
    my_vector.at<double>(r) += m * Jw;
    for(int c=r; c < my_C_inv.rows; c++) {
      my_C_inv.at<double>(r,c) += Jw * Jrow[c];
    }
  }
}


//00197     /// Process all the measurements and compute the weighted least squares set of parameter values
//00198     /// stores the result internally which can then be accessed by calling get_mu()
//00199     void compute(){
//00200
//00201         //Copy the upper right triangle to the empty lower-left.
//00202         for(int r=1; r < my_C_inv.num_rows(); r++)
//00203             for(int c=0; c < r; c++)
//00204                 my_C_inv[r][c] = my_C_inv[c][r];
//00205
//00206         my_decomposition.compute(my_C_inv);
//00207         my_mu=my_decomposition.backsub(my_vector);
//00208     }
//00209

// Process all the measurements and compute the weighted least squares set of parameter values
// stores the result internally which can then be accessed by calling get_mu()
void WLS::Compute()
{
//Copy the upper right triangle to the empty lower-left.
  for(int r=1; r < my_C_inv.rows; r++) {
    for(int c=0; c < r; c++) {
      my_C_inv.at<double>(r,c) = my_C_inv.at<double>(c,r);
    }
  }
  my_decomposition = my_C_inv.inv(cv::DECOMP_CHOLESKY);
  my_mu = my_decomposition * my_vector;
}

