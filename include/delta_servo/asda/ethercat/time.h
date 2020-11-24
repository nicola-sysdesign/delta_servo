#ifndef DELTA_ASDA_ETHERCAT_TIME_H
#define DELTA_ASDA_ETHERCAT_TIME_H
#include <time.h>

#define NSEC_PER_SEC  1000000000


namespace delta { namespace asda { namespace ethercat {


inline void add_timespec(struct timespec *ts, long addtime)
{
   long sec, nsec;
   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;

   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}


inline void diff_timespec(const struct timespec &t1, const struct timespec &t2, struct timespec *t_diff)
{
  if ((t1.tv_nsec - t2.tv_nsec) < 0)
  {
    t_diff->tv_sec = t1.tv_sec - t2.tv_sec - 1;
    t_diff->tv_nsec = t1.tv_nsec - t2.tv_nsec + 1000000000;
  }
  else
  {
    t_diff->tv_sec = t1.tv_sec - t2.tv_sec;
    t_diff->tv_nsec = t1.tv_nsec - t2.tv_nsec;
  }
}

inline double to_sec(const struct timespec &t)
{
  return t.tv_sec + t.tv_nsec / 1000000000.0;
}


inline unsigned long to_nsec(const struct timespec &t)
{
  return t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}

} } } // namespace
#endif
