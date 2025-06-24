#include "logger.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

void log_message(log_level_t lvl, const char *fmt, ...) 
{
    if (lvl > LOG_LEVEL) 
        return;
        
    va_list args; 
    va_start(args, fmt);

    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uart_puts(buf);
}