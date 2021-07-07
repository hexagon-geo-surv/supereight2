#ifndef SE_TIMINGS_HPP
#define SE_TIMINGS_HPP

#include "perfstats.hpp"

#define TICK(str) se::perfstats.sampleDurationStart(str);
#define TICKD(str) se::perfstats.sampleDurationStart(str, true);
#define TOCK(str) se::perfstats.sampleDurationEnd(str);

#endif // SE_TIMINGS_HPP

