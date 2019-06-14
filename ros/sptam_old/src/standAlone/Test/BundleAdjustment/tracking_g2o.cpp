// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <Eigen/StdVector>

#ifdef _MSC_VER
#include <unordered_set>
#else
#include <tr1/unordered_set>
#endif

#include <iostream>
#include <stdint.h>

#include "g2o/config.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
//#include "g2o/types/icp/types_icp.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#elif defined G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif

#include "../../../sptam/types_sba_extension.hpp"



using namespace Eigen;
using namespace std;


const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");


class Sample
{
public:
  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr)
{
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma)
{
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to)
{
  return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform()
{
  return uniform_rand(0., 1.);
}

double Sample::gaussian(double sigma)
{
  return gauss_rand(0., sigma);
}



// calculate stereo projection
void mapPoint(const g2o::SBACam &sbacam, Vector3d &res, const Vector3d &pt3)
{



  Vector4d pt;
  pt.head<3>() = pt3;
  pt(3) = 1.0;
  Vector3d p1 = sbacam.w2i * pt;
  Vector3d p2 = sbacam.w2n * pt;
  Vector3d pb(sbacam.baseline,0,0);

  double invp1 = 1.0/p1(2);
  res.head<2>() = p1.head<2>()*invp1;

  // right camera px
  p2 = sbacam.Kcam*(p2-pb);
  res(2) = p2(0)/p2(2);

}

g2o::OptimizableGraph::Edge* BuildNewStereoEdge
  ( g2o::SparseOptimizer& optimizer
  , g2o::VertexSBAPointXYZ * v_p
  , const size_t camId
  , Vector3d z
  , double PIXEL_NOISE
  , bool ROBUST_KERNEL )
{

  // add noise to the measurement
  z += Vector3d(Sample::gaussian(PIXEL_NOISE),
                Sample::gaussian(PIXEL_NOISE),
                Sample::gaussian(PIXEL_NOISE/16.0));

  g2o::EdgeProjectP2SC * e
      = new g2o::EdgeProjectP2SC();

  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer.vertices().find(camId)->second);

  e->setMeasurement(z);
  e->information() = Matrix3d::Identity();

  if (ROBUST_KERNEL) {
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
  }

  return e;
}

g2o::OptimizableGraph::Edge* BuildNewLeftEdge
  ( g2o::SparseOptimizer& optimizer
  , g2o::VertexSBAPointXYZ * v_p
  , const size_t camId
  , Vector2d z
  , double PIXEL_NOISE
  , bool ROBUST_KERNEL )
{

  // add noise to the measurement
  z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                Sample::gaussian(PIXEL_NOISE));

  g2o::EdgeProjectP2MC * e
      = new g2o::EdgeProjectP2MC();

  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer.vertices().find(camId)->second);

  e->setMeasurement(z);
  e->information() = Matrix2d::Identity();

  if (ROBUST_KERNEL) {
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
  }

  return e;
}

g2o::OptimizableGraph::Edge* BuildNewRightEdge
  ( g2o::SparseOptimizer& optimizer
  , g2o::VertexSBAPointXYZ * v_p
  , const size_t camId
  , Vector2d z
  , double PIXEL_NOISE
  , bool ROBUST_KERNEL )
{

  // add noise to the measurement
  z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                Sample::gaussian(PIXEL_NOISE));

  g2o::EdgeProjectP2MCRight * e
      = new g2o::EdgeProjectP2MCRight();

  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer.vertices().find(camId)->second);

  e->setMeasurement(z);
  e->information() = Matrix2d::Identity();

  if (ROBUST_KERNEL) {
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
  }

  return e;
}



void WriteCovarianceInCSV(Eigen::MatrixXd& covariance_matrix) {

  ofstream file;
  file.open ("covariance.csv");
  file << covariance_matrix.format(CSVFormat);
  file.close();
  return;
}


void WritePointsInCSV(std::vector<Eigen::Vector3d> points) {

  ofstream file;
  file.open ("points.csv");
  for(auto pt : points) {
    file << std::to_string(pt(0)) + "," + std::to_string(pt(1)) + "," + std::to_string(pt(2));
    file << "\n";
  }

  file.close();
  return;
}



int main(int argc, const char* argv[])
{
  if (argc<2)
  {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);

  double OUTLIER_RATIO = 0.0;

  if (argc>2)
  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3)
  {
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4)
  {
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc>5)
  {
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
  cout << "DENSE: "<<  DENSE << endl;



  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
  if (DENSE)
  {
        linearSolver= new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
                cerr << "Using DENSE" << endl;
  }
  else
  {
#ifdef G2O_HAVE_CHOLMOD
        cerr << "Using CHOLMOD" << endl;
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
#elif defined G2O_HAVE_CSPARSE
    linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
        cerr << "Using CSPARSE" << endl;
#else
#error neither CSparse nor Cholmod are available
#endif
  }


  g2o::BlockSolver_6_3 * solver_ptr
      = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  // Generate 3D points
  vector<Vector3d> true_points;

  // number of 3D points to generate
  size_t points_number = 10;



  // Random poins
  for (size_t i=0;i<points_number; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*200,
                                   (Sample::uniform()-0.5)*200,
                                   (Sample::uniform()-0.5)*50+75));
  }

  // selected points
//  true_points.push_back(Vector3d(2,0,10));
//  true_points.push_back(Vector3d(4,0,20));


  // HEIGHT
  size_t image_height = 640; //480;
  size_t image_width = 640;


  // Camera calibration parameters
  Vector2d focal_length(500,500); // pixels
  Vector2d principal_point(image_width/2.0,image_height/2.0); // 640x480 image
  double baseline = 0.075;      // 7.5 cm baseline


  vector<Eigen::Isometry3d, aligned_allocator<Eigen::Isometry3d> > true_poses;

  // Set camera to be tracked
  Vector3d trans(0,0,0);

  Eigen:: Quaterniond q;
  q.setIdentity();
  Eigen::Isometry3d pose;
  pose = q;
  pose.translation() = trans;


  // set the true camera pose
  g2o::SBACam sbacam_true(q, trans);
  sbacam_true.setKcam(
    focal_length[0], focal_length[1],
    principal_point[0], principal_point[1],
    baseline
  );


  // set the camera pose with noise (the camera to be adjusted)
  g2o::SBACam sbacam(q, trans + Vector3d(1,1,1)); // adding noise
  sbacam.setKcam(
    focal_length[0], focal_length[1],
    principal_point[0], principal_point[1],
    baseline
  );



  // add camera to the optimizer
  g2o::VertexCam* v_se3 = new g2o::VertexCam();
  int cam_id = 0; // camera id in optimizer
  v_se3->setId( cam_id );
  v_se3->setFixed(false);
  v_se3->setEstimate( sbacam );

  optimizer.addVertex(v_se3);
  true_poses.push_back(pose);

  int point_id = cam_id + 1;

  cout << endl;
  std::unordered_map<int,int> pointid_2_trueid;

  // add point projections to this vertex
  // usar una tercera parte stereo, otra izquierda y la otra derecha

  for (size_t i=0; i<true_points.size(); ++i)
  {
    g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();


    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setFixed(true); // FIX POINTS!!!
    v_p->setEstimate(true_points.at(i));
//    v_p->setEstimate(true_points.at(i) // se le mete ruido a los puntos
//        + Vector3d(Sample::gaussian(1),
//                   Sample::gaussian(1),
//                   Sample::gaussian(1)));


    // se proyecta el punto sobre la camara y se incrementa el numero de veces
    // que es visto si se encuentra dentro del campo visual de la camara

    // proyectamos y obtenemos una medicion estereo
    Vector3d zStereo;
    mapPoint(sbacam_true, zStereo, true_points.at(i));

    if (zStereo[0]>=0 && zStereo[1]>=0 && zStereo[0]<image_width && zStereo[1]<image_height)
    {

      optimizer.addVertex(v_p);

      // zStereo (xl,y,xr)
      Vector2d zRight(zStereo(2),zStereo(1)); // medicion derecha
      Vector2d zLeft(zStereo(0),zStereo(1)); // medicion izquierda


      g2o::OptimizableGraph::Edge* eStereo = BuildNewStereoEdge(optimizer, v_p, cam_id, zStereo, PIXEL_NOISE, ROBUST_KERNEL);
      g2o::OptimizableGraph::Edge* eLeft = BuildNewLeftEdge(optimizer, v_p, cam_id, zLeft, PIXEL_NOISE, ROBUST_KERNEL);
      g2o::OptimizableGraph::Edge* eRight = BuildNewRightEdge(optimizer, v_p, cam_id, zRight, PIXEL_NOISE, ROBUST_KERNEL);


      optimizer.addEdge(eStereo);
      optimizer.addEdge(eLeft);
      optimizer.addEdge(eRight);


    }

    pointid_2_trueid.insert(make_pair(point_id,i));

    ++point_id;
  } // for de puntos

  cout << endl;
  optimizer.initializeOptimization();

  optimizer.setVerbose(true);

  cout << endl;
  cout << "Performing full BA:" << endl;

  Eigen::Vector3d position_before = v_se3->estimate().translation();
  const Eigen::Quaterniond& orientation_before = v_se3->estimate().rotation();

  std::cout << "BEFORE position: " << position_before << std::endl;


  optimizer.optimize(10);

  cout << endl;


  Eigen::Vector3d position = v_se3->estimate().translation();
  const Eigen::Quaterniond& orientation = v_se3->estimate().rotation();

  std::cout << "AFTER position: " << position << std::endl;


  // GET COVARIANCE
  assert( 0 <= v_se3->hessianIndex() );
  g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;

  if (not optimizer.computeMarginals(spinv, v_se3))
    std::cerr << "WTF" << std::endl;

  Eigen::MatrixXd& cov = *(spinv.block(v_se3->hessianIndex(), v_se3->hessianIndex()));

  std::cout << "COVARIANCE:" << std::endl;
  std::cout << cov << std::endl;


  WriteCovarianceInCSV(cov);

  WritePointsInCSV(true_points);

}
