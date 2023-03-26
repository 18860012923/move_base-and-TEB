#ifndef PORTABLE_UTILS_H
#define PORTABLE_UTILS_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_DRAND48
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
// drand48 返回服从均匀分布的·[0.0, 1.0) 之间的 double 型随机数。
static double drand48(void)
{
    return ((double)rand())/RAND_MAX;
}

static void srand48(long int seedval)
{
    srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif