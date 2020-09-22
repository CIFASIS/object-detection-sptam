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
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "../../../sptam/utils/ProgramOptions.hpp"
#include "../../../sptam/ImageFeatures.hpp"
#include "../../../sptam/utils/Profiler.hpp"

void showFeatures(const std::string& name, const cv::Mat image, const std::vector<cv::KeyPoint>& keypoints)
{
  cv::Mat image_keypoints;

  drawKeypoints( image, keypoints, image_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );

  cv::imshow( name, image_keypoints );
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
  size_t nKeyPoints = 100;

  cv::Ptr<cv::FeatureDetector> featureDetector;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
  featureDetector = cv::xfeatures2d::SURF::create();
  descriptorExtractor = cv::xfeatures2d::SURF::create();

  cv::FileStorage fs("parameters.yaml", cv::FileStorage::READ);
  if( fs.isOpened() ) // if we have file with parameters, read them
  {
    int  valor;
    fs["lala"] >> valor;
    std::cout << "lala: " <<  valor << std::endl;



    cv::FileNode fileNode = fs["FeatureDetector"];
    if ( fileNode.isMap() ) {
      int valor2;
      fileNode["nOctaves"] >> valor2;
      std::cout << "nOctaves: " <<  valor2 << std::endl;
    }

    // Read y Write no funcionan mas!!! no podemos leer los parametros automaticamente, tenemos que hacerlo de manera manual.
    featureDetector->read(fileNode);
    descriptorExtractor->read(fs["parameters"]);
    fs.release();
    cv::Ptr<cv::xfeatures2d::SURF> aux_ptr;
    aux_ptr = featureDetector.dynamicCast<cv::xfeatures2d::SURF>();

    std::cout << "noctaves_param_read: " << aux_ptr->getNOctaves() << std::endl;
    aux_ptr->setNOctaves(3); // lower the contrast threshold, compared to the default value
    std::cout << "noctaves_param_new: " << aux_ptr->getNOctaves()<< std::endl;

    cv::FileStorage fs2("output.yaml", cv::FileStorage::WRITE);
    featureDetector->write(fs2);
    fs2.release();
//    cv::FileStorage fs2;
//    cv::WriteStructContext ws(fs2, "sift_params", CV_NODE_MAP);
//    featureDetector->write(fs);
  }

  std::string name = "SURF";

  cv::Mat image = cv::imread(image_name, 0);

  testImageFeatures(name, image, nKeyPoints, featureDetector, descriptorExtractor);

//  cv::Ptr<cv::Feature2D> briefDescriptorExtractor = cv::xfeatures2d::create("BRIEF"); //(32 );


//  cv::Ptr<cv::FeatureDetector> orbFeatures = cv::FeatureDetector::create( "ORB" );

//  cv::ORB orbFeatures(nKeyPoints, 1.2, 3);
  #ifdef SHOW_FEATURES
    cv::waitKey(0);
  #endif

  return 0;
}
