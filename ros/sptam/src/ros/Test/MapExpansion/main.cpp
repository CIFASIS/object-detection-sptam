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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "GenerateInitMap.h"

using namespace sensor_msgs;
using namespace message_filters;


int main(int argc, char *argv[])
{
  if (argc != 2) {
    std::cout << "Arguments must be passing in this way: " << argv[0] << " <Parameters File>" << std::endl;
    std::cout << std::endl;
    std::cout << "Parameters File: Path to parameters.yml file." << std::endl;
    return -1;
  }

  // Read main arguments
  const char* parametersFileYML = argv[1];

  // Initialize the ROS Handle
  ros::init(argc, argv, "GenerateInitMap");
  ros::NodeHandle nodeHandle("~");

  // Create GenerateInitMap instance
  generateInitMap::GenerateInitMap GenerateInitMap(nodeHandle,parametersFileYML);

  ros::spin();

  ros::waitForShutdown();

  return 0;
}
