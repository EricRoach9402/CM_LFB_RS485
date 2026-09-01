/**
 * @file config_loader.c
 * @brief Load config.json into global_config.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <json-c/json.h>
#include <json-c/json_util.h>
#include <json-c/json_object.h>

#include "config_loader.h"
#include "log.h"

system_config_t global_config;
char            global_config_path[256] = {0};

static const char version[] __attribute__((used)) =
    "VERSION:" CM_LFB_RS485_VERSION;


/**
 * @brief Parse Modbus format from a JSON string ("RTU" or "TCP").
 */
static modbus_format_t parse_modbus_format(const char *format_str)
{
    if (strcmp(format_str, "RTU") == 0) {
        return MODBUS_FORMAT_RTU;
    }
    if (strcmp(format_str, "TCP") == 0) {
        return MODBUS_FORMAT_TCP;
    }
    LOG_ERROR("Invalid Modbus format: %s. Defaulting to RTU.", format_str);
    return MODBUS_FORMAT_RTU;
}

/**
 * @brief Parse modbus_role string and return true when the role is server/master.
 */
static bool parse_modbus_role(const char *modbus_role)
{
    if (strcmp(modbus_role, "Server") == 0 ||
        strcmp(modbus_role, "Master") == 0) {
        return true;
    }
    if (strcmp(modbus_role, "Slave") == 0 ||
        strcmp(modbus_role, "Client") == 0) {
        return false;
    }
    LOG_ERROR("Unknown modbus role: %s. Defaulting to Client/Slave.", modbus_role);
    return false;
}

/**
 * @brief Populate one module_config_t from a JSON object.
 */
static void load_module_config(json_object *json_obj, module_config_t *config)
{
    json_object *field = NULL;

    if (!json_obj) {
        LOG_INFO("Module configuration not found. Skipping.");
        return;
    }

    if (json_object_object_get_ex(json_obj, "name", &field)) {
        strncpy(config->name, json_object_get_string(field),
                sizeof(config->name) - 1);
    }

    if (json_object_object_get_ex(json_obj, "enabled", &field)) {
        config->enabled = json_object_get_boolean(field);
    }

    if (json_object_object_get_ex(json_obj, "modbus_uid", &field)) {
        config->modbus_uid = json_object_get_int(field);
    }

    if (json_object_object_get_ex(json_obj, "modbus_format", &field)) {
        config->format = parse_modbus_format(json_object_get_string(field));
    }

    if (json_object_object_get_ex(json_obj, "modbus_role", &field)) {
        strncpy(config->modbus_role, json_object_get_string(field),
                sizeof(config->modbus_role) - 1);
        config->is_server = parse_modbus_role(config->modbus_role);
    }

    if (json_object_object_get_ex(json_obj, "gpio", &field)) {
        strncpy(config->gpio, json_object_get_string(field),
                sizeof(config->gpio) - 1);
    }

    /* baud_rate/path (RTU) and ip/port (TCP) share storage; only the
     * member matching config->format is meaningful, so only parse that
     * branch to avoid one format's fields clobbering the other's memory. */
    if (config->format == MODBUS_FORMAT_TCP) {
        if (json_object_object_get_ex(json_obj, "ip", &field)) {
            strncpy(config->ip, json_object_get_string(field),
                    sizeof(config->ip) - 1);
        }
        if (json_object_object_get_ex(json_obj, "port", &field)) {
            config->port = json_object_get_int(field);
        }
    } else {
        if (json_object_object_get_ex(json_obj, "path", &field)) {
            strncpy(config->path, json_object_get_string(field),
                    sizeof(config->path) - 1);
        }
        if (json_object_object_get_ex(json_obj, "baud_rate", &field)) {
            config->baud_rate = json_object_get_int(field);
        }
    }

    config->rtu_poll_interval_ms = 200u;
}

/**
 * @brief Load enabled Inverter entries from the JSON "INVERTER" array.
 */
static void load_inverter_config(json_object *inverter_array)
{
    if (!inverter_array ||
        !json_object_is_type(inverter_array, json_type_array)) {
        LOG_WARNING("INVERTER configuration not found or invalid.");
        return;
    }

    int valid_count  = 0;
    int array_length = json_object_array_length(inverter_array);

    for (int i = 0; i < array_length; i++) {
        json_object *inv_obj     = json_object_array_get_idx(inverter_array, i);
        json_object *enabled_obj = NULL;

        if (!json_object_object_get_ex(inv_obj, "enabled", &enabled_obj) ||
            !json_object_get_boolean(enabled_obj)) {
            continue;
        }

        load_module_config(inv_obj, &global_config.inverter[valid_count]);
        valid_count++;

        if (valid_count >= MAX_INVERTER_COUNT) {
            LOG_WARNING("Inverter count exceeds maximum limit (%d). "
                        "Only the first %d enabled entries will be loaded.",
                        MAX_INVERTER_COUNT, MAX_INVERTER_COUNT);
            break;
        }
    }

    global_config.inverter_count = valid_count;
}

/**
 * @brief Load enabled UPS entries from the JSON "UPS" array.
 */
static void load_ups_config(json_object *ups_array)
{
    if (!ups_array || !json_object_is_type(ups_array, json_type_array)) {
        LOG_WARNING("UPS configuration not found or invalid.");
        return;
    }

    int valid_count  = 0;
    int array_length = json_object_array_length(ups_array);

    for (int i = 0; i < array_length; i++) {
        json_object *ups_obj      = json_object_array_get_idx(ups_array, i);
        json_object *enabled_obj  = NULL;

        if (!json_object_object_get_ex(ups_obj, "enabled", &enabled_obj) ||
            !json_object_get_boolean(enabled_obj)) {
            continue;
        }

        load_module_config(ups_obj, &global_config.ups[valid_count]);
        valid_count++;

        if (valid_count >= MAX_UPS_COUNT) {
            LOG_WARNING("UPS count exceeds maximum limit (%d). "
                        "Only the first %d enabled entries will be loaded.",
                        MAX_UPS_COUNT, MAX_UPS_COUNT);
            break;
        }
    }

    global_config.ups_count = valid_count;
}

/**
 * @brief Load and process the JSON configuration file.
 *
 * @param json_path  Path to the JSON configuration file.
 */
void load_json_config(const char *json_path)
{
    json_object *json_config     = NULL;
    json_object *inverter_array  = NULL;
    json_object *ups_array       = NULL;

    json_config = json_object_from_file(json_path);
    if (!json_config) {
        LOG_ERROR("Failed to load JSON configuration file: %s", json_path);
        exit(EXIT_FAILURE);
    }

    strncpy(global_config_path, json_path, sizeof(global_config_path) - 1);
    global_config_path[sizeof(global_config_path) - 1] = '\0';

    memset(&global_config, 0, sizeof(global_config));

    if (json_object_object_get_ex(json_config, "INVERTER", &inverter_array)) {
        load_inverter_config(inverter_array);
    }

    if (json_object_object_get_ex(json_config, "UPS", &ups_array)) {
        load_ups_config(ups_array);
    }

    json_object_put(json_config);

    LOG_INFO("Configuration successfully loaded from %s", json_path);
}
