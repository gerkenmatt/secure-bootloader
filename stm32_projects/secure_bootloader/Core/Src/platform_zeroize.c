#include <stddef.h>
#include "mbedtls/platform.h"   

/* A simple, volatile‐pointer loop that the compiler
 * cannot optimize away, and which does not rely on
 * any function pointers or libc. */
void mbedtls_platform_zeroize( void *buf, size_t len )
{
    volatile unsigned char *p = (volatile unsigned char*) buf;
    while( len-- )
        *p++ = 0;
}

//TODO: move this to ota_crypto.c? 