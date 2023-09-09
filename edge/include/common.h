#ifndef __COMMON_H__
#define __COMMON_H__

#include <sys/time.h>
#include <time.h>

#define ERROR -1
#define OK 0

typedef unsigned char		uchar;
typedef unsigned int		uint;
typedef unsigned long		ulong;
typedef unsigned long long	ulonglong;
typedef unsigned short		ushort;
typedef void *				PTR;
typedef unsigned short		Bool;
typedef int (*Fxn)();           /* generic function type */
typedef void                    *Ptr;           /* pointer to arbitrary type */
typedef char                    *String;

// #define XERROR(tag, format, ...) {printf("[%32s] <%s:%d>...." format, tag, __FUNCTION__, __LINE__, ## __VA_ARGS__);}
#define XLOG_TRACE                                          1
#define XLOG_DEBUG                                          2
#define XLOG_INFO                                           3
#define XLOG_WARN                                           4
#define XLOG_ERROR                                          5
#define LOG_FATAL                                           6
#define ELEVEL	                                            7

#define XDEBUG(format, ...)	{\
    if (ELEVEL > XLOG_DEBUG) { \
        struct timeval    tv; \
        struct tm         *tm_ptr; \
        gettimeofday(&tv, NULL); \
        tm_ptr = localtime(&tv.tv_sec);\
        printf("[%d-%02d-%02d %02d:%02d:%02d.%03ld] [DEBUG] ", 1900+tm_ptr->tm_year, 1+tm_ptr->tm_mon, \
            tm_ptr->tm_mday, tm_ptr->tm_hour, tm_ptr->tm_min, tm_ptr->tm_sec, tv.tv_usec/1000); \
        printf("<%-36s> <%-20s:%-4d> ...... " format, __FILE__, __FUNCTION__, __LINE__, ## __VA_ARGS__); \
    }\
}

#define XINFO(format, ...)	{\
    if (ELEVEL > XLOG_INFO) { \
        struct timeval    tv; \
        struct tm         *tm_ptr; \
        gettimeofday(&tv, NULL); \
        tm_ptr = localtime(&tv.tv_sec);\
        printf("[%d-%02d-%02d %02d:%02d:%02d.%03ld] [INFO]] ", 1900+tm_ptr->tm_year, 1+tm_ptr->tm_mon, \
            tm_ptr->tm_mday, tm_ptr->tm_hour, tm_ptr->tm_min, tm_ptr->tm_sec, tv.tv_usec/1000); \
        printf("<%-36s> <%-20s:%-4d> ...... " format, __FILE__, __FUNCTION__, __LINE__, ## __VA_ARGS__); \
    }\
}

#define XERROR(format, ...)	{\
    if (ELEVEL > XLOG_ERROR) { \
        struct timeval    tv; \
        struct tm         *tm_ptr; \
        gettimeofday(&tv, NULL); \
        tm_ptr = localtime(&tv.tv_sec);\
        printf("[%d-%02d-%02d %02d:%02d:%02d.%03ld] [ERROR] ", 1900+tm_ptr->tm_year, 1+tm_ptr->tm_mon, \
            tm_ptr->tm_mday, tm_ptr->tm_hour, tm_ptr->tm_min, tm_ptr->tm_sec, tv.tv_usec/1000); \
        printf("<%-36s> <%-20s:%-4d> ...... " format, __FILE__, __FUNCTION__, __LINE__, ## __VA_ARGS__); \
    } \
}

#endif 