/**
 * @file syscalls.c
 * @brief Minimal syscall stubs for newlib on bare‐metal
 *
 * These stubs allow heap allocation and minimal I/O support for bare-metal.
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include "uart.h"

// Symbols from linker script
extern char _end;      // End of data/bss
extern char _estack;   // Top of RAM
extern char _heap_end; // Optional heap boundary (can alias _estack)

static char *heap_ptr = NULL;

caddr_t _sbrk(int incr)
{
    if (!heap_ptr)
        heap_ptr = (char *)(((uintptr_t)&_end + 7) & ~7);  // Align on first use

    char *prev = heap_ptr;
    char *next = heap_ptr + incr;

    if (next > &_heap_end) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_ptr = next;
    return (caddr_t)prev;
}

caddr_t _sbrk_r(struct _reent *r, ptrdiff_t incr)
{
    (void)r;
    return _sbrk(incr);
}

void *calloc(size_t nmemb, size_t size)
{
    if (!heap_ptr)
        heap_ptr = (char *)(((uintptr_t)&_end + 7) & ~7);  // Align on first use

    size_t total = nmemb * size;
    total = (total + 7) & ~7;  // 8-byte align

    char *prev = heap_ptr;
    char *next = heap_ptr + total;

    if (next > &_heap_end) {
        errno = ENOMEM;
        return NULL;
    }

    for (size_t i = 0; i < total; i++) {
        prev[i] = 0;
    }

    heap_ptr = next;
    return prev;
}

void free(void *ptr) {

    // No-op: we don’t support freeing memory in this simple allocator
    (void)ptr;
}


int _write(int f, const void *b, size_t n) { return n; }
int _close(int f) { return -1; }
int _fstat(int f, struct stat *s) { s->st_mode = S_IFCHR; return 0; }
int _isatty(int f) { return 1; }
off_t _lseek(int f, off_t o, int w) { return 0; }
int _read(int f, void *b, size_t n) { return 0; }
void _exit(int c) { while (1); }
int _kill(int p, int s) { return -1; }
int _getpid(void) { return 1; }
int _open(const char *pathname, int flags, int mode) {
    (void)pathname; (void)flags; (void)mode;
    return -1;
}
