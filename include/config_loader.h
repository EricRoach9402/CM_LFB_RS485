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
    CONNECTION_DISCONNECTED = 0,
    CONNECTION_CONNECTED = 1,
    CONNECTION_UNKNOWN = 99
} connection_state_t;

typedef enum {
    MODBUS_FORMAT_RTU,
    MODBUS_FORMAT_TCP,
} modbus_format_t;

/**
 * @brief Per-module configuration loaded from config.json.
 * RTU uses path/baud_rate; TCP uses ip/port (union storage).
 * connection_state is updated at runtime and mirrored to the shared pool.
 */
typedef struct {
    char name[64];
    bool enabled;
    int modbus_uid;
    modbus_format_t format;
    char path[256];
    union {
        struct { int baud_rate; };
        struct { char ip[64]; int port; };
    };
    connection_state_t connection_state;
} module_config_t;

typedef struct {
    module_config_t inverter[MAX_INVERTER_COUNT];
    int inverter_count;
    module_config_t ups[MAX_UPS_COUNT];
    int ups_count;
} system_config_t;

extern system_config_t global_config;
extern char global_config_path[256];

/** Module callback table used by polling threads. */
typedef struct {
    int (*init_callback)(module_config_t *config);
    int (*start_callback)(const module_config_t *config);
    int (*process_callback)(module_config_t *config);
    int (*error_callback)(module_config_t *config, int connection_state);
    int (*msg_callback)(module_config_t *config, uint16_t addr,
                        uint16_t *values, size_t count);
} module_callbacks_t;

/**
 * @brief Load config.json into global_config.
 * @param json_path Path to the JSON file.
 */
void load_json_config(const char *json_path);

#endif /* CONFIG_LOADER_H */
