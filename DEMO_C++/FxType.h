#ifndef FX_FXTYPE_H_
#define FX_FXTYPE_H_

#if defined(_WIN64) || defined(__WIN64__) || defined(WIN64)
#define FX_PLATFORM_WIN64 1
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define FX_PLATFORM_WIN32 1
#elif defined(__linux__) || defined(__linux)
#if defined(__x86_64__) || defined(__aarch64__) || defined(__LP64__)
#define FX_PLATFORM_LINUX64 1
#else
#define FX_PLATFORM_LINUX32 1
#endif
#else
#error "Unsupported platform! Only Windows (32/64) and Linux (32/64) are supported."
#endif

#define FX_VOID void
#define FX_BOOL unsigned char
#define FX_TRUE 1
#define FX_FALSE 0

#define FX_CHAR char
#define FX_UCHAR unsigned char

#define FX_INT8 signed char
#define FX_UINT8 unsigned char

#define FX_INT16 short
#define FX_UINT16 unsigned short

#define FX_INT32 int
#define FX_UINT32 unsigned int

#define FX_INT64 long long
#define FX_UINT64 unsigned long long

#define FX_FLOAT float
#define FX_DOUBLE double

#if defined(FX_PLATFORM_WIN32) || defined(FX_PLATFORM_WIN64) || defined(FX_PLATFORM_LINUX32)
#define FX_INT32L long
#define FX_UINT32L unsigned long
#elif defined(FX_PLATFORM_LINUX64)
#define FX_INT32L long
#define FX_UINT32L unsigned long
#endif

#endif /* FX_FXTYPE_H_ */
