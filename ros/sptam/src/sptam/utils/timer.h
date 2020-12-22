#pragma once

#include <chrono>

namespace sptam
{
  class Timer
  {
    public:
      Timer(void);

      void start(void); /* measures initial time */
      void stop(void); /* measures elapsed time */
      double elapsed(void) const; /* returns elapsed time in seconds */

      static double now(void); /* returns seconds since epoch */

    private:
      typedef std::chrono::high_resolution_clock clock_t;
      std::chrono::time_point<clock_t> t;
      double elapsed_seconds;
  };
}

std::ostream& operator<< (std::ostream& stream, const sptam::Timer& t);

