#ifndef BBC_COMMON_H
#define BBC_COMMON_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BBC_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BBC_MIN(a,b) ((a) < (b) ? (a) : (b))
#define BBC_MAX(a,b) ((a) > (b) ? (a) : (b))

static inline float bbc_clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline int32_t bbc_clampi32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
#endif
