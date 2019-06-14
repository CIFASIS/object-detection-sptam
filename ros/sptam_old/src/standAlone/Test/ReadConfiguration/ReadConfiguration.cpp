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
#include <stdio.h>
#include "../../Localization/MotionModel.h"
#include "../../StereoProcessing/StereoProcessing.h"


int main(int argc, char *argv[])
{
  if (argc < 1 || argc > 2) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Parameters File> " << std::endl;
    std::cout << std::endl;
    std::cout << "Parameters File: Path to parameters.yml file." << std::endl;
    return -1;
  }

  const char* parametersFileYML = argv[1];

  // Read the YML file
  CameraCalibration cameraCalibrationLeft;
  CameraCalibration cameraCalibrationRight;

  RectifyMaps rectifyMaps;
  ReadCalibration(parametersFileYML, cameraCalibrationLeft, cameraCalibrationRight);

  std::cout << cameraCalibrationLeft.GetImageWidth() << std::endl;
  std::cout << cameraCalibrationLeft.GetImageHeight() << std::endl;
  std::cout << cameraCalibrationLeft.GetPixelSizeInX() << std::endl;
  std::cout << cameraCalibrationLeft.GetPixelSizeInY() << std::endl;
  std::cout << cameraCalibrationLeft.GetIntrinsic() << std::endl;
  std::cout << cameraCalibrationLeft.GetDistortion() << std::endl;
  std::cout << cameraCalibrationLeft.GetRotation() << std::endl;
  std::cout << cameraCalibrationLeft.GetTranslation() << std::endl;

  std::cout << cameraCalibrationRight.GetImageWidth() << std::endl;
  std::cout << cameraCalibrationRight.GetImageHeight() << std::endl;
  std::cout << cameraCalibrationRight.GetPixelSizeInX() << std::endl;
  std::cout << cameraCalibrationRight.GetPixelSizeInY() << std::endl;
  std::cout << cameraCalibrationRight.GetIntrinsic() << std::endl;
  std::cout << cameraCalibrationRight.GetDistortion() << std::endl;
  std::cout << cameraCalibrationRight.GetRotation() << std::endl;
  std::cout << cameraCalibrationRight.GetTranslation() << std::endl;
  return 0;
}


