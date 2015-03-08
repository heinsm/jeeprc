#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

#define __unused( x )   ( (void) x )

#define NUM_OF(x)               (sizeof (x) / sizeof *(x))

#define return_if_not( _cond, _code ) do { \
    if ( ! (_cond) ) return _code; \
} while(0)

#define return_if( _cond, _code ) do { \
    if ( (_cond) ) return _code; \
} while(0)

#if !defined( EOK )  //see http://www.ibm.com/developerworks/aix/library/au-errnovariable/
#  define EOK 0         /* no error */
#endif

typedef enum
{
    verblvl_none,
    verblvl_regular,
    verblvl_more,
    verblvl_moremore,
    verblvl_moremoremore,
} debug_lvl_t;

/*
 * Initialize utilities
 */
int util_init();

/*
 * De-initialize utilities
 */

int util_fini();

/*
 * Get process verbosity level
 */
debug_lvl_t get_verbose_lvl( void );

/*
 * Set process verbosity level
 */
void set_verbose_lvl(debug_lvl_t verbose_lvl);

/*
 * Print to message with specified verbosity level
 */
int printlvl_stdout(debug_lvl_t lvl, const char *__format, ...);

/*
 * Print to message always
 * ( except on when verbosity level is set to verblvl_none )
 */
int print_stdout(const char *__format, ...);

/*
 * Print to error with specified verbosity level
 */
int printlvl_stderr(debug_lvl_t lvl, const char *__format, ...);

/*
 * Print to error always
 * ( except on when verbosity level is set to verblvl_none )
 */
int print_stderr(const char *__format, ...);

#endif
