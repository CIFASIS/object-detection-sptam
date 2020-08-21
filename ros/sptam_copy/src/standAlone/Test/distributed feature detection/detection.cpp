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

#define SHOW_FEATURES

#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "../../../sptam/utils/ProgramOptions.hpp"
#include "../../../sptam/ImageFeatures.hpp"
#include "../../../sptam/utils/Profiler.hpp"

void showFeatures(const std::string& name, const cv::Mat image, const std::vector<cv::KeyPoint>& keypoints)
{
  cv::Mat image_keypoints;

  drawKeypoints( image, keypoints, image_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

  cv::imshow( name, image_keypoints );
}


void testGoodFeatures(const std::string& name, const cv::Mat image, size_t nKeyPoints)
{
  // image size: 1241 x 376 (px)
  // ratio 1241/376 = 3.300531914893617
  // con minDistance = 15
  // (1) width / minDistance = 82.73333333333333
  // (2) height / minDistance = 25.066666666666666
  // luego la cantidad máx posible de puntos es (1)*(2)
  //   = 2073.8488888888887
  // como definimos minDistance en función de la cantidad esperada
  // (maxCorners) de puntos y un margen decente hardcodeado?

  /**
   * int maxCorners
   *   Maximum number of corners to return. If there are more corners
   *   than are found, the strongest of them are returned.
   *   Default: 1000
   *
   * double qualityLevel
   *   Parameter characterizing the minimal accepted quality
   *   of image corners. The parameter value is multiplied by the best
   *   corner quality measure. The corners with the quality measure
   *   less than the product are rejected.
   *   Default: 0.01
   *
   * double minDistance
   *   Minimum possible Euclidean distance between the returned corners.
   *   Default: 1
   *
   * int blockSize
   *   Size of an average block for computing a derivative
   *   covariation matrix over each pixel neighborhood.
   *   Default: 3
   *
   * bool useHarrisDetector
   *   Default: false
   *
   * double k
   *   Free parameter of the Harris detector.
   *   Default: 0.04
   */
  cv::GoodFeaturesToTrackDetector detector(nKeyPoints, 0.01, 15., 3, false);

  std::vector<cv::KeyPoint> keypoints;

  double start = GetSeg();
  detector.detect( image, keypoints );
  double end = GetSeg();

  //~ std::cout << name << " detected keypoints: " << keypoints.size() << std::endl;
  //~ std::cout << name << ": " << end - start << " (s)" << std::endl;
  std::cout << name << " " << keypoints.size() << " " << end - start << std::endl;

  #ifdef SHOW_FEATURES
  showFeatures(name, image, keypoints);
  #endif
}

void testImageFeatures(const std::string& name, const cv::Mat image, size_t nKeyPoints,
                       const cv::Ptr<cv::FeatureDetector>& featureDetector, const cv::Ptr<cv::DescriptorExtractor>& descriptorExtractor)
{
  size_t matchingCellSize = 30;

  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<size_t> indexes;

  double start = GetSeg();

  // extract features
  featureDetector->detect(image,keypoints);

  descriptorExtractor->compute(image,keypoints,descriptors);


  /**
   * size_t MatchingCellSize
   *   ...
   * 
   * size_t CreationCellSize
   *   ...
   */
  ImageFeatures imageFeatures(image.size(), keypoints, descriptors, matchingCellSize);

  /**
   * const size_t nKeyPoints
   * 
   * std::vector<cv::KeyPoint>& keyPoints
   * 
   * cv::Mat& descriptors
   * 
   * std::vector<size_t>& indexes);
   * 
   */
//  imageFeatures.GetEvenlyDistributedKeyPoints(nKeyPoints, keypoints, descriptors, indexes);
//  imageFeatures.GetUnmatchedKeyPoints(keypoints, descriptors, indexes);


  double end = GetSeg();

  //~ std::cout << name << " detected keypoints: " << keypoints.size() << std::endl;
  //~ std::cout << name << ": " << end - start << " (s)" << std::endl;
  std::cout << name << " " << keypoints.size() << " " << end - start << std::endl;

  #ifdef SHOW_FEATURES
  showFeatures(name, image, keypoints);
  #endif
}

int main(int argc, char **argv)
{
  std::string image_name;

  // Parse program options

  std::string app_name = boost::filesystem::basename( argv[0] );

  ProgramOptions program_options( app_name );

  program_options.addPositionalArgument<std::string>("image", "input image filename", image_name);

  program_options.addOptionalArgumentFlag("help,h", "Display this help message");
  program_options.addOptionalArgumentFlag("version,v", "Display the version number");

  try
  {
    program_options.parse(argc, argv);

    if( program_options.count("help") ) {
      std::cout << program_options << std::endl;
      return EXIT_SUCCESS;
    }
    else if( program_options.count("version") ) {
      std::cout << "program version: 1.0" << std::endl;
      return EXIT_SUCCESS;
    }

    program_options.notify();
  }
  catch(boost::program_options::required_option& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << program_options << std::endl;
    return EXIT_FAILURE;
  } 
  catch(boost::program_options::error& e) 
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << program_options << std::endl;
    return EXIT_FAILURE;
  }

  // TODO hardcoded
  size_t nKeyPoints = 1000;

  /**
   * maxSize
   *   default: 16
   * 
   * responseThreshold
   *   default: 30
   * 
   * lineThresholdProjected
   *   default: 10
   * 
   * lineThresholdBinarized
   *   default: 8
   * 
   * suppressNonmaxSize
   *   default: 5
   */
  cv::Ptr<cv::FeatureDetector> starFeatureDetector = cv::FeatureDetector::create( "STAR" ); //( 16, 20 );

  cv::Ptr<cv::FeatureDetector> starPyramidFeatureDetector = new cv::PyramidAdaptedFeatureDetector (starFeatureDetector, 3);

  /**
   * int threshold
   *   default: 1
   *
   * bool nonmaxSuppression
   *   default: true
   *
   * type
   *   default: FastFeatureDetector::TYPE_9_16
   */
  cv::Ptr<cv::FeatureDetector> fastFeatureDetector = cv::FeatureDetector::create( "FAST" ); //( 40 );

  /**
   * int bytes
   *   default: 32
   */
  cv::Ptr<cv::DescriptorExtractor> briefDescriptorExtractor = cv::DescriptorExtractor::create("BRIEF"); //(32 );

  /**
   * 
   * int nfeatures
   *   The maximum number of features to retain.
   *   default: 500
   *
   * float scaleFactor
   *   Pyramid decimation ratio, greater than 1. scaleFactor==2 means
   *   the classical pyramid, where each next level has 4x less pixels
   *   than the previous, but such a big scale factor will degrade
   *   feature matching scores dramatically. On the other hand,
   *   too close to 1 scale factor will mean that to cover certain
   *   scale range you will need more pyramid levels and so the speed
   *   will suffer.
   *   default: 1.2f
   *
   * int nlevels
   *   The number of pyramid levels. The smallest level
   *   will have linear size equal to
   *   input_image_linear_size/pow(scaleFactor, nlevels).
   *   default 8
   *
   * int edgeThreshold
   *   This is size of the border where the features are not detected.
   *   It should roughly match the patchSize parameter.
   *   default: 31
   *
   * int firstLevel
   *   It should be 0 in the current implementation.
   *   default: 0
   *
   * int WTA_K
   *   The number of points that produce each element of the oriented
   *   BRIEF descriptor. The default value 2 means the BRIEF
   *   where we take a random point pair and compare their brightnesses,
   *   so we get 0/1 response. Other possible values are 3 and 4.
   *   For example, 3 means that we take 3 random points (of course,
   *   those point coordinates are random, but they are generated
   *   from the pre-defined seed, so each element of BRIEF descriptor
   *   is computed deterministically from the pixel rectangle),
   *   find point of maximum brightness and output index of the winner
   *   (0, 1 or 2). Such output will occupy 2 bits, and therefore
   *   it will need a special variant of Hamming distance, denoted as
   *   NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4
   *   random points to compute each bin (that will also occupy 2 bits
   *   with possible values 0, 1, 2 or 3).
   *   default: 2
   *
   * int scoreType
   *   The default HARRIS_SCORE means that Harris algorithm is used
   *   to rank features (the score is written to KeyPoint::score and
   *   is used to retain best nfeatures features); FAST_SCORE
   *   is alternative value of the parameter that produces slightly
   *   less stable keypoints, but it is a little faster to compute.
   *   default: ORB::HARRIS_SCORE
   *
   * int patchSize
   *   size of the patch used by the oriented BRIEF descriptor.
   *   Of course, on smaller pyramid layers the perceived image area
   *   covered by a feature will be larger.
   *   default: 31
   */

  cv::Ptr<cv::FeatureDetector> orbFeatures = cv::FeatureDetector::create( "ORB" );

  cv::Ptr<cv::FeatureDetector> orbGridFeatureDetector = new cv::GridAdaptedFeatureDetector (orbFeatures);


//  cv::ORB orbFeatures(nKeyPoints, 1.2, 3);



  // image size: 1241 x 376 (px)
  // ratio 1241/376 = 3.300531914893617
  // con minDistance = 15
  // (1) width / minDistance = 82.73333333333333
  // (2) height / minDistance = 25.066666666666666
  // luego la cantidad máx posible de puntos es (1)*(2)
  //   = 2073.8488888888887
  // como definimos minDistance en función de la cantidad esperada
  // (maxCorners) de puntos y un margen decente hardcodeado?

  /**
   * int maxCorners
   *   Maximum number of corners to return. If there are more corners
   *   than are found, the strongest of them are returned.
   *   Default: 1000
   *
   * double qualityLevel
   *   Parameter characterizing the minimal accepted quality
   *   of image corners. The parameter value is multiplied by the best
   *   corner quality measure. The corners with the quality measure
   *   less than the product are rejected.
   *   Default: 0.01
   *
   * double minDistance
   *   Minimum possible Euclidean distance between the returned corners.
   *   Default: 1
   *
   * int blockSize
   *   Size of an average block for computing a derivative
   *   covariation matrix over each pixel neighborhood.
   *   Default: 3
   *
   * bool useHarrisDetector
   *   Default: false
   *
   * double k
   *   Free parameter of the Harris detector.
   *   Default: 0.04
   */
  cv::Ptr<cv::FeatureDetector> gfttFeatureDetector = cv::FeatureDetector::create( "GFTT" ); //(nKeyPoints, 0.01, 15., 3, false);


  cv::Ptr<cv::FeatureDetector> gfttPyramidFeatureDetector = new cv::PyramidAdaptedFeatureDetector (gfttFeatureDetector);


  cv::Ptr<cv::FeatureDetector> gfttGridFeatureDetector = new cv::GridAdaptedFeatureDetector (gfttFeatureDetector);

  cv::Ptr<cv::FeatureDetector> gfttGridPyramidFeatureDetector = new cv::PyramidAdaptedFeatureDetector (gfttGridFeatureDetector);



  cv::Mat image;/* = cv::imread( image_name );*/

  cv::VideoCapture capture( image_name );

  // check if we succeeded
  if( not capture.isOpened() ) {
    std::cerr << "could not open capture device " << image_name << std::endl;
    return EXIT_FAILURE;
  }

  while( capture.read( image ) )
  {

    // GFTT
    testGoodFeatures("goodFeat", image, nKeyPoints);
    testImageFeatures("gftt/brief", image, nKeyPoints, gfttFeatureDetector, briefDescriptorExtractor);
    testImageFeatures("Grid_gftt/brief", image, nKeyPoints, gfttGridFeatureDetector, briefDescriptorExtractor);

    testImageFeatures("Pyramid_gftt/brief", image, nKeyPoints, gfttPyramidFeatureDetector, briefDescriptorExtractor);
    testImageFeatures("Grid_Pyramid_gftt/brief", image, nKeyPoints, gfttGridPyramidFeatureDetector, briefDescriptorExtractor);

    // FAST
    //testImageFeatures("fast/brief", image, nKeyPoints, fastFeatureDetector, briefDescriptorExtractor);

    // STAR
    testImageFeatures("star/brief", image, nKeyPoints, starFeatureDetector, briefDescriptorExtractor);
    testImageFeatures("Pyramid_star/brief", image, nKeyPoints, starPyramidFeatureDetector, briefDescriptorExtractor);

    // ORB
    testImageFeatures("orb/orb", image, nKeyPoints, orbFeatures, orbFeatures);
    testImageFeatures("Grid_orb/orb", image, nKeyPoints, orbGridFeatureDetector, orbFeatures);


    #ifdef SHOW_FEATURES
    cv::waitKey( 10 );
    #endif
  }

  #ifdef SHOW_FEATURES
  cv::waitKey( 0 );
  #endif

	return 0;
}
