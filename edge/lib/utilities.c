#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <syslog.h>
#include "utilities.h"

#define LOG_STRFMT_LEN              (10)
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void string_sort(char *arr, int len)
{
    int i, j, temp;
    for (i = 0; i < len - 1; i++)
        for (j = 0; j < len - 1 - i; j++)
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
}

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( uint32_t seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

int8_t Nibble2HexChar( uint8_t a )
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}

uint32_t Crc32( uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT - 0x04C11DB7
    const uint32_t reversedPolynom = 0xEDB88320;

    // CRC initial value
    uint32_t crc = 0xFFFFFFFF;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint32_t )buffer[i];
        for( uint16_t i = 0; i < 8; i++ )
        {
            crc = ( crc >> 1 ) ^ ( reversedPolynom & ~( ( crc & 0x01 ) - 1 ) );
        }
    }

    return ~crc;
}

uint32_t Crc32Init( void )
{
    return 0xFFFFFFFF;
}

uint32_t Crc32Update( uint32_t crcInit, uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT - 0x04C11DB7
    const uint32_t reversedPolynom = 0xEDB88320;

    // CRC initial value
    uint32_t crc = crcInit;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint32_t )buffer[i];
        for( uint16_t i = 0; i < 8; i++ )
        {
            crc = ( crc >> 1 ) ^ ( reversedPolynom & ~( ( crc & 0x01 ) - 1 ) );
        }
    }
    return crc;
}

uint32_t Crc32Finalize( uint32_t crc )
{
    return ~crc;
}

void hex_dump(const void *src, int length, int line_size,
                     char *prefix)
{
    int i = 0;
    const unsigned char *address = src;
    const unsigned char *line = address;
    unsigned char c;

    printf("%s | ", prefix);
    while (length-- > 0) {
        printf("%02X ", *address++);
        if (!(++i % line_size) || (length == 0 && i % line_size)) {
            if (length == 0) {
                while (i++ % line_size)
                    printf("__ ");
            }
            printf(" | ");  /* right close */
            while (line < address) {
                c = *line++;
                printf("%c", (c < 33 || c > 177) ? 0x2E : c);
            }
            printf("\n");
            if (length > 0)
                printf("%s | ", prefix);
        }
    }
}

/* use this carefully, unless you known what you are doing */
char *hex_dump_to_buf(const void *src, int length, int line_size, char *prefix)
{
    int i = 0, len;
    const unsigned char *address = src;
    const unsigned char *line = address;
    unsigned char c;
    /* careful this */
    static char dump_buf[1024 * 4] = {0};

    len = snprintf(dump_buf, sizeof(dump_buf), "%s | ", prefix);
    while (length-- > 0 && len < sizeof(dump_buf)) {
        len += snprintf(dump_buf + len, sizeof(dump_buf), "%02X ", *address++);
        if (!(++i % line_size) || (length == 0 && i % line_size)) {
            if (length == 0) {
                while (i++ % line_size)
                    len += snprintf(dump_buf + len, sizeof(dump_buf), "__ ");
            }
            len += snprintf(dump_buf + len, sizeof(dump_buf), " | ");  /* right close */
            while (line < address) {
                c = *line++;
                len += snprintf(dump_buf + len, sizeof(dump_buf), "%c", (c < 33 || c > 177) ? 0x2E : c);
            }
            if (length > 0) {
                len += snprintf(dump_buf + len, sizeof(dump_buf), "\n");
                len += snprintf(dump_buf + len, sizeof(dump_buf), "%s | ", prefix);
            }
        }
    }

    dump_buf[len] = '\0';

    return dump_buf;
}

int log_print(int priority, char *fmt, ...)
{
    int i = 0, d, ret, len, j;
    char c, *s;
    uint8_t *hbuf;
    double f;
    char strfmt[LOG_STRFMT_LEN+2];
    va_list ap;

    if (isatty(fileno(stdout))) {
        switch (priority) {
            case LOG_CRIT:
                //foregroud white, background red
                printf("\033[37;41;1m");
                break;
            case LOG_ERR:
                // red
                printf("\033[31m");
                break;
            case LOG_WARNING:
                // yellow
                printf("\033[33m");
                break;
            case LOG_NOTICE:
                // green
                printf("\033[32;2m");
                break;
            case LOG_INFO:
                // bold
                printf("\033[1m");
                break;
            case LOG_DEBUG:
            default:
                //printf("\033[32m");
                break;
        }
    }

    if (fmt != NULL) {
        va_start(ap, fmt);
        i = 0;
        while (*fmt) {
            if (*fmt == '%') {
                strfmt[0] = '%';
                j=1;
                while ((fmt[j]>='0' && fmt[j]<='9') ||
                        (fmt[j]== '-') || (fmt[j]== '+') || (fmt[j]== '.')) {
                    strfmt[j] = fmt[j];
                    j++;
                    if (j == LOG_STRFMT_LEN) {
                        break;
                    }
                }
                strfmt[j] = fmt[j];
                fmt += j;
                j++;
                strfmt[j] = '\0';

                switch (*fmt) {
                    case '%':
                        ret = printf(strfmt);
                        i+=ret;
                        break;
                    case 'd':
                        d = va_arg(ap, int);
                        ret = printf(strfmt, d);
                        i+=ret;
                        break;
                    case 'u':
                        d = va_arg(ap, int);
                        ret = printf(strfmt, (uint32_t)d);
                        i+=ret;
                        break;
                    case 'x':
                    case 'X':
                        d = va_arg(ap, int);
                        ret = printf(strfmt, d);
                        i+=ret;
                        break;
                    case 'h':
                    case 'H':
                        hbuf = va_arg(ap, uint8_t *);
                        len = va_arg(ap, int);
                        for (d = 0; d < len; d++) {
                            if (*fmt == 'h') {
                                ret = printf("%02X", hbuf[d]);
                            } else {
                                ret = printf("%02X ", hbuf[d]);
                            }
                            i+=ret;
                        }
                        break;
                    case 's':
                        s = va_arg(ap, char *);
                        ret = printf(strfmt, s);
                        i+=ret;
                        break;
                    case 'c':
                        c = (char)va_arg(ap, int);
                        ret = printf(strfmt, c);
                        i+=ret;
                        break;
                    case 'f':
                        f = va_arg(ap, double);
                        ret = printf(strfmt, f);
                        i+=ret;
                        break;
                }
                fmt++;
            } else {
                fputc(*fmt++, stdout);
                i++;
            }
        }
        va_end(ap);
    }

    if (isatty(fileno(stdout))) {
        printf("\033[0m");
    }

    printf("\n");
    fflush(stdout);
    i++;

    return i;
}

