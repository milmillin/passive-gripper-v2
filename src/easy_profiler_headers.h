#ifdef COMPILE_WITH_EASY_PROFILER  // This should be set by CMake

#include <easy/profiler.h>

#else  // #ifdef COMPILE_WITH_EASY_PROFILER

#ifndef EASY_PROFILER_H
// Use empty macros/functions when easy profiler is disabled
#define EASY_PROFILER_ENABLE
#define EASY_FUNCTION(...)
#define EASY_BLOCK(...)
#define EASY_END_BLOCK
#define EASY_MAIN_THREAD
#define EASY_EVENT(...)

namespace profiler {
  inline void dumpBlocksToFile(char *) {}

  enum EasyBlockStatus : uint8_t {
      OFF = 0, ///< The block is OFF
      ON = 1, ///< The block is ON (but if it's parent block is off recursively then this block will be off too)
      FORCE_ON = ON | 2, ///< The block is ALWAYS ON (even if it's parent has turned off all children)
      OFF_RECURSIVE = 4, ///< The block is OFF and all of it's children by call-stack are also OFF.
      ON_WITHOUT_CHILDREN = ON | OFF_RECURSIVE, ///< The block is ON but all of it's children are OFF.
      FORCE_ON_WITHOUT_CHILDREN = FORCE_ON | OFF_RECURSIVE, ///< The block is ALWAYS ON but all of it's children are OFF.
  };
}

#endif  // #ifndef EASY_PROFILER_H
#endif  // #ifdef COMPILE_WITH_EASY_PROFILER

#define EASY_STATEMENT(name, statement) EASY_BLOCK(name);statement;EASY_END_BLOCK;
