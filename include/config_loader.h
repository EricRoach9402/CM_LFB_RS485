/**
 * @file config_loader.h
 * @brief JSON configuration types and loader.
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CM_LFB_RS485_VERSION "0.0.1"
#define MAX_INVERTER_COUNT 10
#define MAX_UPS_COUNT 10

typedef enum {
    MODBUS_FORMAT_RTU,
    MODBUS_FORMAT_TCP,
} modbus_format_t;

/**
 * @brief Per-module configuration loaded from config.json.
 * RTU and TCP transport fields share union storage (path/baud_rate vs ip/port).
 */
typedef struct {
    char name[64];
    bool enabled;
    int modbus_uid;
    modbus_format_t format;
    union {
        struct { char path[256]; int baud_rate; };
        struct { char ip[64]; int port; };
    };
} module_config_t;

typedef struct {
    module_config_t inverter[MAX_INVERTER_COUNT];
    int inverter_count;
    module_config_t ups[MAX_UPS_COUNT];
    int ups_count;
} system_config_t;

extern system_config_t global_config;
extern char global_config_path[256];

/** Module callback table used by polling threads. @a unit is module runtime state. */
typedef struct {
    int (*init_callback)(void *unit);
    int (*start_callback)(const void *unit);
    int (*process_callback)(void *unit);
    int (*error_callback)(void *unit, int connection_state);
    int (*msg_callback)(void *unit, uint16_t addr,
                        uint16_t *values, size_t count);
} module_callbacks_t;

/**
 * @brief Load config.json into global_config.
 * @param json_path Path to the JSON file.
 */
void load_json_config(const char *json_path);

#endif /* CONFIG_LOADER_H */
