#include "logger.h"
#include "uart.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Static Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Converts a log level to its string representation.
 * @param lvl The log level to convert.
 * @return A string representation of the log level.
 */
static const char* get_level_string(log_level_t lvl);

// -----------------------------------------------------------------------------
// Public Function Implementations
// -----------------------------------------------------------------------------

void log_message(log_level_t lvl, const char *fmt, ...)
{
    // 1. Filter out messages based on the compile-time log level.
    if (lvl > LOG_LEVEL) 
        return;

    // This buffer will hold the final, fully-formatted log message.
    char final_buf[256]; 
    
    // This temporary buffer holds just the user's formatted message part.
    char user_buf[128];

    // 2. Format the user's variable arguments into the temporary buffer.
    va_list args;
    va_start(args, fmt);
    int user_len = vsnprintf(user_buf, sizeof(user_buf), fmt, args);
    va_end(args);

    // Optional: Check for truncation of the user's message
    if (user_len >= sizeof(user_buf)) 
    {
        // If truncated, copy "..." to the last 3 characters to indicate it.
        strcpy(user_buf + sizeof(user_buf) - 4, "...");
    }

    // 3. Create the final log message with timestamp and level prefix.
    int len = snprintf(final_buf, sizeof(final_buf), "[%08u] [%-5s] %s\r\n",
             (unsigned int)get_systick(), // Timestamp
             get_level_string(lvl),             // Log level string
             user_buf);                         // The user's message

    // 4. Send the fully formatted string to the UART.
    // Check for final truncation, though less likely with the larger buffer.
    if (len > 0 && len < sizeof(final_buf)) 
        uart_puts(final_buf);
    else 
        uart_puts("Log message truncated!\r\n");
    
}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static const char* get_level_string(log_level_t lvl)
{
    switch (lvl) {
        case LOG_LEVEL_ERROR: return "ERROR";
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_INFO:  return "INFO";
        case LOG_LEVEL_DEBUG: return "DEBUG";
        default:              return "NONE";
    }
}
