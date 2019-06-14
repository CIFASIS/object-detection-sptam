#include <iomanip>
#include <ios>
#include "timer.h"

sptam::Timer::Timer(void) :
  elapsed_seconds(0)
{

}

void sptam::Timer::start(void)
{
  t = clock_t::now();
}

void sptam::Timer::stop(void)
{
  elapsed_seconds = std::chrono::duration<double, std::milli>(clock_t::now() - t).count() * 1e-3;
}

double sptam::Timer::elapsed(void) const
{
  return elapsed_seconds;
}

double sptam::Timer::now()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(clock_t::now().time_since_epoch()).count() * 1e-6;
}


std::ostream& operator<< (std::ostream& stream, const sptam::Timer& t)
{
  stream << std::setprecision(16) << std::fixed << t.elapsed();
  return stream;
}
