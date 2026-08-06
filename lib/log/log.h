#ifndef LOG_H
#define LOG_H

/**
 * @file log.h
 * @brief Header file for the logging system in CM_Modbus.
 *
 * This file provides a logging interface with support for multiple log levels
 * (VERBOSE, DEBUG, INFO, WARNING, ERROR, ASSERT).
 */

#include <stdio.h>
#include <stdarg.h>

// Enumeration for log levels
typedef enum {
    LOG_LEVEL_VERBOSE = 0, // Display all messages, most detailed level
    LOG_LEVEL_DEBUG   = 1, // Debugging messages
    LOG_LEVEL_INFO    = 2, // General information messages
    LOG_LEVEL_WARNING = 3, // Warning messages indicating potential issues
    LOG_LEVEL_ERROR   = 4, // Error messages indicating serious problems
    LOG_LEVEL_ASSERT  = 5  // Assert messages for critical failures
} log_level_t;

// Function declarations
void set_log_level(log_level_t level); // Set the current log level
void log_message(log_level_t level, const char *format, ...); // Log a message

// Convenience macros for different log levels
#define LOG_VERBOSE(fmt, ...) log_message(LOG_LEVEL_VERBOSE, "[%s] " fmt, __FILE__, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)   log_message(LOG_LEVEL_DEBUG, "[%s:%s] " fmt, __FILE__, __func__, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    log_message(LOG_LEVEL_INFO, "[%s] " fmt, __FILE__, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) log_message(LOG_LEVEL_WARNING, "[%s] " fmt, __FILE__, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   log_message(LOG_LEVEL_ERROR, "[%s:%s] " fmt, __FILE__, __func__, ##__VA_ARGS__)
#define LOG_ASSERT(fmt, ...)  log_message(LOG_LEVEL_ASSERT, "[%s:%s] " fmt, __FILE__, __func__, ##__VA_ARGS__)

#endif // LOG_H
