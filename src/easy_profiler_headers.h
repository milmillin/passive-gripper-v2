#ifdef COMPILE_WITH_EASY_PROFILER  // This should be set by CMake

#include <easy/profiler.h>

#else  // #ifdef COMPILE_WITH_EASY_PROFILER

// Use empty macros/functions when easy profiler is disabled
#define EASY_PROFILER_ENABLE
#define EASY_FUNCTION(...)
#define EASY_BLOCK(...)

namespace profiler {
  inline void dumpBlocksToFile(char *) {}
}

#endif  // #ifdef COMPILE_WITH_EASY_PROFILER
