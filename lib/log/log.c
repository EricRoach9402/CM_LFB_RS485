#include "log.h"
#include <time.h>
#include <sys/time.h>
#include <string.h>

#define PROJECT_NAME "CM_MODBUS"

// Global variable to hold the current log level
static log_level_t current_log_level = LOG_LEVEL_INFO; // Default log level is INFO

/**
 * @brief Set the current log level.
 *
 * This function sets the current log level, controlling which messages will be displayed.
 *
 * @param level The desired log level.
 */
void set_log_level(log_level_t level) {
    current_log_level = level;
}

/**
 * @brief Log a message with a specified log level.
 *
 * This function logs a message if the log level is greater than or equal to
 * the current log level. ERROR and ASSERT messages are always logged.
 *
 * @param level The log level of the message.
 * @param format The format string (similar to printf).
 * @param ... Additional arguments for the format string.
 */
void log_message(log_level_t level, const char *format, ...) {
    // Always log ERROR and ASSERT messages, otherwise respect current log level
    if (level < current_log_level && level < LOG_LEVEL_ERROR) {
        return;
    }

    va_list args;
    va_start(args, format);

    const char *level_str = NULL;
    FILE *output = stdout;

    // Add timestamp with milliseconds
    char time_buffer[40];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;
    struct tm *tm_info = localtime(&now);
    int millisec = tv.tv_usec / 1000;
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", tm_info);
    snprintf(time_buffer + strlen(time_buffer), sizeof(time_buffer) - strlen(time_buffer), ".%03d", millisec);

    switch (level) {
        case LOG_LEVEL_VERBOSE:
            level_str = "VERBOSE";
            break;
        case LOG_LEVEL_DEBUG:
            level_str = "DEBUG";
            break;
        case LOG_LEVEL_INFO:
            level_str = "INFO";
            break;
        case LOG_LEVEL_WARNING:
            level_str = "WARNING";
            break;
        case LOG_LEVEL_ERROR:
            level_str = "ERROR";
            output = stderr;
            break;
        case LOG_LEVEL_ASSERT:
            level_str = "ASSERT";
            output = stderr;
            break;
        default:
            level_str = "UNKNOWN";
    }

    fprintf(output, "[%s] [%s] [%s]: ", time_buffer, PROJECT_NAME, level_str);
    vfprintf(output, format, args);
    fprintf(output, "\n");

    va_end(args);
}
