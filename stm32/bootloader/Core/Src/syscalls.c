/**
 * @file syscalls.c
 * @brief Minimal syscall stubs for newlib on bare‐metal
 *
 * These stubs allow heap allocation and minimal I/O support for bare-metal.
 */
#include "uart.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------
// Forward Declarations 
// -----------------------------------------------------------------------------

// These symbols are defined by the linker script to delineate memory regions.
extern char _end;       // Represents the end of the .data and .bss sections in RAM. This is the start of the heap.
extern char _estack;    // Points to the top of the main stack in RAM. Used as the upper bound for the heap.
extern char _heap_end;  // An optional symbol, usually aliasing _estack, defining the practical end of the heap.

// -----------------------------------------------------------------------------
// Module-Private Data (Static Globals)
// -----------------------------------------------------------------------------

// tracks the current end of the heap segment. It moves upwards as memory is allocated.
static char *heap_ptr = NULL;

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

caddr_t _sbrk(int incr)
{
    // Initialize heap_ptr on the first call.
    // It's set to the address immediately after the .bss segment (`_end`),
    // aligned to an 8-byte boundary for proper memory access.
    if (!heap_ptr) 
    {
        heap_ptr = (char *)(((uintptr_t)&_end + 7) & ~7);
    }

    char *prev = heap_ptr;          // Store the current heap break (start of new block).
    char *next = heap_ptr + incr;   // Calculate the new heap break after increment.

    // Check if the allocation request would exceed the available RAM for the heap.
    // `_heap_end` (or `_estack`) defines the upper limit of usable RAM.
    if (next > &_heap_end) 
    {
        errno = ENOMEM; // Set error code to indicate "Out of memory".
        return (caddr_t)-1; // Return -1 to signal failure, as per _sbrk contract.
    }

    heap_ptr = next;    // Update the heap break to the new end.
    return (caddr_t)prev; // Return the address of the newly allocated block.
}

caddr_t _sbrk_r(struct _reent *r, ptrdiff_t incr)
{
    // This is a reentrant version of _sbrk, required by newlib for thread safety.
    // In this bare-metal, single-threaded context, it simply calls the non-reentrant version.
    (void)r; // Cast to void to suppress unused parameter warning.
    return _sbrk(incr);
}

void *calloc(size_t nmemb, size_t size)
{
    // Initialize heap_ptr if this is the first memory allocation.
    // This ensures the heap starts correctly aligned after static data.
    if (!heap_ptr) 
    {
        heap_ptr = (char *)(((uintptr_t)&_end + 7) & ~7);
    }

    // Calculate the total memory required and ensure it's 8-byte aligned.
    // Alignment is important for performance and to meet architecture-specific requirements.
    size_t total = nmemb * size;
    total = (total + 7) & ~7;

    char *prev = heap_ptr;          // Mark the current heap break as the start of the new block.
    char *next = heap_ptr + total;  // Calculate the new heap break.

    // Check for heap overflow. If the requested block extends beyond the heap limit,
    // the allocation fails.
    if (next > &_heap_end) 
    {
        errno = ENOMEM; // Set errno to indicate out of memory.
        return NULL;    // Return NULL to signal allocation failure.
    }

    // Zero-fill the newly allocated memory. This is a distinguishing feature of calloc.
    for (size_t i = 0; i < total; i++) 
    {
        prev[i] = 0;
    }

    heap_ptr = next;    // Update the heap break to the new end.
    return prev;        // Return the pointer to the zero-initialized memory block.
}

void free(void *ptr)
{
    // TODO: Implement proper memory management (e.g., a linked list of free blocks).
    // This is a no-op implementation, meaning memory is never returned to the heap.
    // In a simple embedded system, this might be acceptable if memory usage is fixed
    // or if the system reboots frequently enough.
    (void)ptr; // Cast to void to suppress unused parameter warning, as ptr is not used.
}
int _write(int f, const void *b, size_t n) 
{ 
    return n; 
}

int _close(int f) 
{ 
    return -1; 
}

int _fstat(int f, struct stat *s) 
{ 
    s->st_mode = S_IFCHR; 
    return 0; 
}

int _isatty(int f) 
{ 
    return 1; 
}

off_t _lseek(int f, off_t o, int w) 
{ 
    return 0; 
}

int _read(int f, void *b, size_t n) 
{ 
    return 0; 
}

void _exit(int c) 
{ 
    while (1); 
}

int _kill(int p, int s) 
{ 
    return -1; 
}

int _getpid(void) 
{ 
    return 1; 
}

int _open(const char *pathname, int flags, int mode) 
{
    (void)pathname; 
    (void)flags; 
    (void)mode;
    return -1;
}
