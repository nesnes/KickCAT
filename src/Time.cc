#include <ctime>
#include <cstdio>
#include <cerrno>
/// \brief OS agnostic time API implementation

#include "Time.h"

namespace kickcat
{
    nanoseconds since_epoch()
    {
        auto now = time_point_cast<nanoseconds>(system_clock::now());
        return now.time_since_epoch();
    }


    nanoseconds elapsed_time(nanoseconds start)
    {
        return since_epoch() - start;
    }

    nanoseconds getElapsedTime(nanoseconds startTime)
    {
        return (getCurrentTime() - startTime);
    }


    nanoseconds getCurrentTime()
    {
#ifdef WDC_TARGET_GAZEBO
        return getCurrentSharedTime();
#else
        auto now = time_point_cast<nanoseconds>(system_clock::now());
        return now.time_since_epoch();
#endif
    }
}
