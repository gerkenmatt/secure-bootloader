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
    // Filter out messages if their level is higher (less severe) than the configured LOG_LEVEL.
    // This allows compile-time control over which log messages are actually processed.
    if (lvl > LOG_LEVEL)
        return;

    // `final_buf` will store the complete log message, including timestamp and level prefix.
    char final_buf[256];
    // `user_buf` is a temporary buffer to format the user-provided message arguments.
    char user_buf[128];

    // Format the user's variable arguments into `user_buf`.
    // `vsnprintf` is used for safety to prevent buffer overflow.
    va_list args;
    va_start(args, fmt);
    int user_len = vsnprintf(user_buf, sizeof(user_buf), fmt, args);
    va_end(args);

    // Check if the user's formatted message was truncated due to buffer size limits.
    if (user_len >= sizeof(user_buf))
    {
        // If truncated, add "..." at the end to clearly indicate incomplete message.
        // Subtract 4 to account for the null terminator and three dots.
        strcpy(user_buf + sizeof(user_buf) - 4, "...");
    }

    // Compose the final log message by combining timestamp, log level string, and the user's message.
    // The timestamp is obtained from SysTick for high-resolution timing.
    int len = snprintf(final_buf, sizeof(final_buf), "[%08u] [%-5s] %s\r\n",
                       (unsigned int)get_systick(),      // Timestamp (e.g., milliseconds since boot)
                       get_level_string(lvl),            // String representation of the log level
                       user_buf);                        // The user's formatted log content

    // Send the complete log message via UART.
    // Check `len` to ensure `snprintf` did not encounter an error or truncate the `final_buf`.
    if (len > 0 && len < sizeof(final_buf))
        uart_puts(final_buf);
    else
        // Fallback message if the final log message itself was too long or an error occurred.
        uart_puts("Log message truncated!\r\n");

}

// -----------------------------------------------------------------------------
// Static Function Implementations
// -----------------------------------------------------------------------------

static const char* get_level_string(log_level_t lvl)
{
    // Return a fixed string literal corresponding to each log level enum value.
    // This provides a human-readable prefix for log messages.
    switch (lvl) {
        case LOG_LEVEL_ERROR: return "ERROR";
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_INFO:  return "INFO";
        case LOG_LEVEL_DEBUG: return "DEBUG";
        default:              return "NONE"; // Handle any undefined log levels.
    }
}