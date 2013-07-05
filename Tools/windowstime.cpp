#include "../DataStructures/TimingUtil.h"
#ifdef _MSC_VER
void gettimeofday(struct timeval* t,void* timezone)
 {       struct _timeb timebuffer;
        _ftime( &timebuffer );
        t->tv_sec=timebuffer.time;
        t->tv_usec=1000*timebuffer.millitm;
 }
#endif
