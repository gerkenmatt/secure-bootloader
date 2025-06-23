#include "logger.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

static const char *log_level_strings[] = { "NONE", "ERROR", "WARN", "INFO", "DEBUG" };

void logger_init(void) 
{
    // Any future logger-specific initialization can go here.
}

void logger_log(log_level_t lvl, const char *fmt, ...) 
{
    if (lvl < LOG_LEVEL) 
        return;
        
    va_list args; 
    va_start(args, fmt);

    char buf[128];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    uart_puts(buf);
}