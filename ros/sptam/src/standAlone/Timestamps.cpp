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

#include "Timestamps.hpp"

#include <fstream>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#define SECONDS_TO_NANOSECONDS 1000000000

ros::Time getSystemTime()
{
  boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration duration = t - boost::posix_time::from_time_t( 0 );

  return ros::Time( duration.total_seconds(), duration.total_nanoseconds() );
}

Timestamps::Timestamps(const double rate, size_t frame_ini)
  : constant_rate_(true), rate_(rate), next_time_( frame_ini * rate ), last_time_update_( 0 )
{}

Timestamps::Timestamps(const std::string& filename, size_t frame_ini)
  : constant_rate_(false), next_frame_( frame_ini ), last_time_update_( 0 )
{
  std::ifstream file( filename );

  if ( not file.is_open() )
    throw std::invalid_argument("Error opening timestamps file " + filename);
  else
    std::cout << "Parsing timestamps file " + filename << std::endl;

  for(std::string line; std::getline(file, line); )
  {
    std::istringstream linestream( line );

    double timestamp;
    linestream >> timestamp;

    times_.push_back( timestamp );
  }

  std::cout << "Loaded " << times_.size() << " timestamps poses" << std::endl;
}

ros::Time Timestamps::getNextWhenReady()
{
  ros::Time next_time = constant_rate_ ? next_time_ : ros::Time( times_.at( next_frame_ ) );
  ros::Time current_time = getSystemTime();

  #ifndef SINGLE_THREAD
  // Compute time remainder so we don't go too fast, sleep if necessary
  if ( ros::Time(0) != last_time_update_ )
  {
    double elapsed = ( current_time - last_time_update_ ).toSec();
    double cycle = ( next_time - last_time_ ).toSec();
    double to_sleep = cycle - elapsed;

    if ( to_sleep < 0 )
      std::cerr << "WARNING tracking is slower than the camera feed by " << 1+to_sleep << " (s)" << std::endl;
    else
    {
      boost::this_thread::sleep_for( boost::chrono::nanoseconds( (int)(to_sleep * SECONDS_TO_NANOSECONDS) ) );
    }
  }
  #endif

  if ( constant_rate_ )
    next_time_ += ros::Duration( rate_ );
  else
    next_frame_++;

  last_time_ = next_time;
  last_time_update_ = current_time;

  return next_time;
}
