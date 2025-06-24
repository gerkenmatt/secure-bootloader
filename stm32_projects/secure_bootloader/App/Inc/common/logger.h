/**
 * @file logger.h
 * @brief Logging interface for the STM32 bootloader project.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdarg.h>

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define LOG_ERROR(...)  logger_log(LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARN(...)   logger_log(LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_INFO(...)   logger_log(LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOG_DEBUG(...)  logger_log(LOG_LEVEL_DEBUG, __VA_ARGS__)

#define LOG_LEVEL LOG_LEVEL_DEBUG ///< Compile-time log level

// -----------------------------------------------------------------------------
// Enumerations
// -----------------------------------------------------------------------------

/**
 * @brief Log level enumeration.
 */
typedef enum {
    LOG_LEVEL_NONE,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} log_level_t;

// -----------------------------------------------------------------------------
// Public Function Prototypes
// -----------------------------------------------------------------------------

/**
 * @brief Initializes the logger.
 */
void logger_init(void);

/**
 * @brief Logs a formatted message at the specified log level.
 * @param lvl Log level
 * @param fmt Format string
 * @param ... Variable arguments
 */
void logger_log(log_level_t lvl, const char *fmt, ...);

#endif // LOGGER_H