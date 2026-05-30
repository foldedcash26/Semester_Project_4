/*
 * rtwtypes.h — ESP32/ESP-IDF compatible replacement
 * Replaces the MATLAB-generated version that depends on tmwtypes.h
 * (which is a MATLAB host-only file not available on embedded targets).
 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Signed integer types */
typedef int8_t    int8_T;
typedef int16_t   int16_T;
typedef int32_t   int32_T;
typedef int64_t   int64_T;

/* Unsigned integer types */
typedef uint8_t   uint8_T;
typedef uint16_t  uint16_T;
typedef uint32_t  uint32_T;
typedef uint64_t  uint64_T;

/* Floating point types */
typedef float     real32_T;
typedef double    real_T;

/* Boolean type */
typedef bool      boolean_T;

/* Character type */
typedef char      char_T;

/* Byte type */
typedef uint8_t   byte_T;

#endif /* RTWTYPES_H */
