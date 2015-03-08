#include "util.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/types.h>

#define STDPRINT_NAME                       __FILE__ ":"
#define PRINT_BUF_MAXSIZE                   1024

static debug_lvl_t                          verbose_lvl = verblvl_none;

int util_init()
{
    return EOK;
}

int util_fini()
{
    return EOK;
}

debug_lvl_t get_verbose_lvl( void )
{
    return verbose_lvl;
}

void set_verbose_lvl(debug_lvl_t __verbose_lvl)
{
    verbose_lvl = __verbose_lvl;
}

int printlvl_stdout(debug_lvl_t lvl, const char *__format, ...)
{
    if (verbose_lvl >= lvl)
    {
        char buf[PRINT_BUF_MAXSIZE];

        //process formatted string into buffer one time
        va_list argp;
        va_start( argp, __format );
        vsnprintf( buf, PRINT_BUF_MAXSIZE, __format, argp );

        //print the formatted buffer
        return fprintf( stdout, buf );
    }

    return -1;
}

int print_stdout(const char *__format, ...)
{
    if (verbose_lvl)
    {
        char buf[PRINT_BUF_MAXSIZE];

        //process formatted string into buffer one time
        va_list argp;
        va_start( argp, __format );
        vsnprintf( buf, PRINT_BUF_MAXSIZE, __format, argp );

        //print the formatted buffer
        return fprintf( stdout, buf );
    }

    return -1;
}

int printlvl_stderr(debug_lvl_t lvl, const char *__format, ...)
{
    if (verbose_lvl >= lvl)
    {
        char buf[PRINT_BUF_MAXSIZE];

        //process formatted string into buffer one time
        va_list argp;
        va_start( argp, __format );
        vsnprintf( buf, PRINT_BUF_MAXSIZE, __format, argp );

        //print the formatted buffer
        return fprintf( stdout, buf );
    }

    return -1;
}

int print_stderr(const char *__format, ...)
{
    if (verbose_lvl)
    {
        char buf[PRINT_BUF_MAXSIZE];

        //process formatted string into buffer one time
        va_list argp;
        va_start( argp, __format );
        vsnprintf( buf, PRINT_BUF_MAXSIZE, __format, argp );

        //print the formatted buffer
        return fprintf( stdout, buf );
    }

    return -1;
}
