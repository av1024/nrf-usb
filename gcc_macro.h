
#define INLINE   inline __attribute__((__always_inline__))
#define NOINLINE __attribute__((__noinline__))

// https://www.mikrocontroller.net/topic/65923#530326
#define PRELOAD(reg,var) \
  __asm__ __volatile (";PRELOAD " reg " with " #var : "=" reg (var) : "0" (var))
