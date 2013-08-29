#ifndef __GCC_MACRO__
#define __GCC_MACRO__

#define INLINE   inline __attribute__((__always_inline__))
#define NOINLINE __attribute__((__noinline__))

// https://www.mikrocontroller.net/topic/65923#530326
/*
 Note! May grow optimized code depend on compiler switch
 Usage:
 StructA *ptr = &struct;
 PRELOAD("y", ptr);
 ptr->a ...
*/
#define PRELOAD(reg,var) \
  __asm__ __volatile (";PRELOAD " reg " with " #var : "=" reg (var) : "0" (var))

// avoid -Wunised-variable def
#define UNUSED __attribute__((__unused__))

#define __nop() do {__asm__ __volatile__("nop"); } while(0)

#endif // __GCC_MACRO__
