/**
 * @file syscalls.c
 * @brief Minimal syscall stubs for newlib on bare‐metal
 *
 * On a Cortex-M without an operating system, newlib’s C library expects
 * certain low-level functions (_sbrk, _write, _open, etc.) to be provided
 * by the OS. This file implements the bare minimum stubs so that:
 *   - _sbrk allows a simple heap (malloc/free) to grow from the end of RAM
 *   - _write/_read/_open/_close/_lseek/_fstat/_isatty satisfy I/O calls
 *   - _exit/_kill/_getpid provide termination hooks
 *
 * These stubs become necessary once you link in libraries (like mbedtls)
 * that pull in newlib routines, which in turn reference OS syscalls that
 * don’t exist on bare-metal. By providing no-op or trivial implementations,
 * you satisfy the linker and enable heap usage without an underlying OS.
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>


extern char _end;
static char *heap_end;

caddr_t _sbrk(int incr) { char *p = heap_end?:&_end; heap_end = p+incr; return (caddr_t)p; }
int _write(int f,const void*b,size_t n){return n;}
int _close(int f){return -1;}
int _fstat(int f, struct stat*s){s->st_mode=S_IFCHR;return 0;}
int _isatty(int f){return 1;}
off_t _lseek(int f,off_t o,int w){return 0;}
int _read(int f,void*b,size_t n){return 0;}
void _exit(int c){while(1);}
int _kill(int p,int s){return -1;}
int _getpid(void){return 1;}
int _open(const char *pathname, int flags, int mode) {
    (void)pathname; (void)flags; (void)mode;
    return -1;
}