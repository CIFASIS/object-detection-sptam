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
#include "Map.hpp"
#include "utils/macros.hpp"

#ifdef SHOW_PROFILING
#include "utils/log/Profiler.hpp"
#endif

#include <list>

namespace sptam
{

/*void Map::RemoveBadPoints()
{
  #ifdef SHOW_PROFILING
    double start = GetSeg();
  #endif

  #ifdef SHOW_PROFILING
    double end = GetSeg();
    WriteToLog(" ba remove_bad_points_lock: ", start, end);
  #endif

  // CAUTION: all points are checked!!!
  std::list<sptam::Map::SharedPoint> bad_points;
  for ( sptam::Map::Point& mapPoint : graph_.mapPoints() )
    if ( mapPoint.IsBad() )
      bad_points.push_back( mapPoint );

  for ( sptam::Map::Point& mapPoint : bad_points )
    graph_.removeMapPoint( mapPoint );
}*/

} // namespace sptam
