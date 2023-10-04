#pragma once


#if __has_include(<klee/klee.h>)
#  include <klee/klee.h>
#else
#  define klee_int( param ) 1
#  define klee_make_symbolic( p1, p2, p3 )
#  define klee_assume( param ) assert(param)
#endif

#if defined(USE_KLEE)
#  if !__has_include(<klee/klee.h>)
#    warning USE_KLEE defined, but no klee include available
#  endif
#  define INFO( param )
#else
#  include <iostream>
#  define INFO( param ) param
#endif

