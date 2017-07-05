#ifndef MACROS_H_BY_CYCLOPS
#define MACROS_H_BY_CYCLOPS

#include <stdio.h>
#include <math.h>

extern FILE *shmacros_log;
extern int shmacros_log_users;

#ifndef NDEBUG
#define SH_DEBUG_MODE
#endif

#ifndef SHMACROS_LOG_PREFIX
// Define this macro before including shmacros.h to have this prefix in the log file. Otherwise, "Unknown Module" will be used.
#warning Define SHMACROS_LOG_PREFIX to a string before including shmacros.h file to have that prefix in the logs.
#define SHMACROS_LOG_PREFIX		"Unknown Module"
#endif

#ifdef SH_DEBUG_MODE
// LOG is an fprintf with i arguments apart from file name and format string, includes \n
#define LOG(x, ...)			do { fprintf(shmacros_log, SHMACROS_LOG_PREFIX": "x"\n", ##__VA_ARGS__); fflush(shmacros_log); } while (0)
#else
// LOG is an fprintf with i arguments apart from file name and format string, includes \n
#define LOG(x, ...)			fprintf(shmacros_log, SHMACROS_LOG_PREFIX": "x"\n", ##__VA_ARGS__)
#endif

// Vector operations
#define DOT(x, y)			((x)[0]*(y)[0]+(x)[1]*(y)[1]+(x)[2]*(y)[2])
#define NORM2(x)			DOT((x), (x))
#define NORM(x)				(sqrt(NORM2(x)))
#define CROSS_X(x, y)			((x)[1]*(y)[2]-(x)[2]*(y)[1])
#define CROSS_Y(x, y)			((x)[2]*(y)[0]-(x)[0]*(y)[2])
#define CROSS_Z(x, y)			((x)[0]*(y)[1]-(x)[1]*(y)[0])
#define CROSS(x, y)			(CROSS_X(x,y)),(CROSS_Y(x,y)),(CROSS_Z(x,y))
#define DISTANCE2(x, y)			(((x)[0]-(y)[0])*((x)[0]-(y)[0])\
					+((x)[1]-(y)[1])*((x)[1]-(y)[1])\
					+((x)[2]-(y)[2])*((x)[2]-(y)[2]))
#define DISTANCE(x, y)			(sqrt(DISTANCE2((x), (y))))
#define NORMALIZED_CROSS(x, y, d)	(CROSS_X(x,y))/(d),(CROSS_Y(x,y))/(d),(CROSS_Z(x,y))/(d)

// Scalar operations
#define MAX(x, y)			(((x)>(y))?(x):(y))
#define MIN(x, y)			(((x)<(y))?(x):(y))
#define EQUAL(x, y)			((x)-(y) < 1e-6 && (y)-(x) < 1e-6)
#define IS_ZERO(x)			((x) < 1e-6 && (x) > 1e-6)

#endif
