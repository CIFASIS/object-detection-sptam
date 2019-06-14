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

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "../../Core/ImageProcessing.h"


cv::Mat GetPatch(const cv::Mat& image, const uint& px, const uint& py, const uint& patchSize)
{

 int topLeftCornerX = px - ((patchSize-1) /2);
 int topLeftCornerY = py - ((patchSize-1) /2);

 cv::Mat tmp = image(cv::Rect(topLeftCornerX,topLeftCornerY,patchSize,patchSize));
 cv::Mat patch;

 tmp.copyTo(patch);

 return patch;
}

cv::Mat GetPatch(const cv::Mat& image, const cv::Point2d& point2d, const uint& patchSize)
{

 int topLeftCornerX = point2d.x - ((patchSize-1) /2);
 int topLeftCornerY = point2d.y - ((patchSize-1) /2);

 cv::Mat tmp = image(cv::Rect(topLeftCornerX,topLeftCornerY,patchSize,patchSize));
 cv::Mat patch;

 tmp.copyTo(patch);

 return patch;

int main(int argc, char *argv[])
{
  if (argc < 2 || argc > 2) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Input Image>" << std::endl;
    return -1;
  }

  const char* filename = argv[1];

  cv::Mat image;
  image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the Image

  if(!image.data ) {                             // Check for invalid input
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  unsigned int neighborhoodSize = 50;

  cv::Mat patch = GetPixelNeighborhood(image, 200, 200, neighborhoodSize);

  const std::string windowInputImage = "Input image";
  cv::namedWindow(windowInputImage, 1);

  cv::imshow(windowInputImage, image);

  const std::string windowPatch = "Patch";
  cv::namedWindow(windowPatch, 1);

  cv::imshow(windowPatch, patch);
  cv::imwrite("template.png", patch);

  cv::waitKey(0);

  return 0;
}
