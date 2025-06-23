#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdarg.h>

// -----------------------------------------------------------------------------
// Log Levels
// -----------------------------------------------------------------------------

typedef enum {
    LOG_LEVEL_NONE,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} log_level_t;

// -----------------------------------------------------------------------------
//  Configuration
// -----------------------------------------------------------------------------

// Set the compile-time log level.
// Messages with a level lower than this will not be compiled.
#define LOG_LEVEL LOG_LEVEL_DEBUG

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the logger.
 */
void logger_init(void);

void logger_log(log_level_t lvl, const char *fmt, ...);

// -----------------------------------------------------------------------------
// Logging Macros
// -----------------------------------------------------------------------------

#define LOG_ERROR(...)  logger_log(LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARN(...)   logger_log(LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_INFO(...)   logger_log(LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOG_DEBUG(...)  logger_log(LOG_LEVEL_DEBUG, __VA_ARGS__)


#endif // LOGGER_H