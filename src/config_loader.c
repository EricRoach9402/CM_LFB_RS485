/**
 * @file config_loader.c
 * @brief Load config.json into global_config.
 */

#include <json-c/json.h>
#include <json-c/json_object.h>
#include <json-c/json_util.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_loader.h"
#include "log.h"

#define MODBUS_UID_MIN 1
#define MODBUS_UID_MAX 247
#define RTU_BAUD_MIN 4800
#define RTU_BAUD_MAX 115200

system_config_t global_config;
char global_config_path[256] = {0};

static const char version[] __attribute__((used)) =
    "VERSION:" CM_LFB_RS485_VERSION;

static bool parse_modbus_format_value(const char *format_str,
                                      modbus_format_t *out);
static bool str_is_empty(const char *s);
static bool validate_rtu_baud(int baud);
static void config_fail_missing(const char *family, int json_index,
                                const module_config_t *cfg,
                                const char *field);
static void config_fail_empty(const char *family, int json_index,
                              const module_config_t *cfg,
                              const char *field);
static void config_fail_range(const char *family, int json_index,
                              const module_config_t *cfg,
                              const char *field, const char *range);
static void config_fail_invalid(const char *family, int json_index,
                                const module_config_t *cfg,
                                const char *field, const char *expected);
static void config_load_fail(const char *family, int json_index,
                             const module_config_t *cfg, const char *reason);
static bool json_has_field(json_object *json_obj, const char *field_name);
static void validate_module_config(const char *family, int json_index,
                                   json_object *json_obj,
                                   const module_config_t *cfg);
static void load_module_config(const char *family, int json_index,
                               json_object *json_obj, module_config_t *config);
static void load_inverter_config(json_object *inverter_array);
static void load_ups_config(json_object *ups_array);

/**
 * @brief Load config.json into global_config.
 * @param json_path Path to the JSON file.
 */
void load_json_config(const char *json_path)
{
    json_object *json_config = NULL;
    json_object *inverter_array = NULL;
    json_object *ups_array = NULL;

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

/**
 * @brief Parse Modbus format string.
 * @param format_str Format string from JSON.
 * @param out Parsed format on success.
 * @return true if format_str is RTU or TCP.
 */
static bool parse_modbus_format_value(const char *format_str,
                                      modbus_format_t *out)
{
    if (!format_str || !out) {
        return false;
    }
    if (strcmp(format_str, "RTU") == 0) {
        *out = MODBUS_FORMAT_RTU;
        return true;
    }
    if (strcmp(format_str, "TCP") == 0) {
        *out = MODBUS_FORMAT_TCP;
        return true;
    }
    return false;
}

/**
 * @brief Return true when s is NULL or empty.
 * @param s String to test.
 * @return true if empty.
 */
static bool str_is_empty(const char *s)
{
    return (s == NULL || s[0] == '\0');
}

/**
 * @brief Return true when baud is within supported RTU range.
 * @param baud Baud rate from config.
 * @return true if valid.
 */
static bool validate_rtu_baud(int baud)
{
    return (baud >= RTU_BAUD_MIN && baud <= RTU_BAUD_MAX &&
            (baud % 100) == 0);
}

/**
 * @brief Return true when json_obj contains field_name.
 * @param json_obj Source JSON object.
 * @param field_name Field key to look up.
 * @return true if present.
 */
static bool json_has_field(json_object *json_obj, const char *field_name)
{
    json_object *field = NULL;

    return json_object_object_get_ex(json_obj, field_name, &field);
}

/**
 * @brief Log a config error and terminate the process.
 * @param family Config section name (INVERTER or UPS).
 * @param json_index Index in the JSON array.
 * @param cfg Parsed module config (may be partially filled).
 * @param reason Human-readable failure reason.
 */
static void config_load_fail(const char *family, int json_index,
                             const module_config_t *cfg, const char *reason)
{
    const char *name = "(unnamed)";

    if (cfg && cfg->name[0] != '\0') {
        name = cfg->name;
    }

    LOG_ERROR("[config] %s[%d] '%s': %s", family, json_index, name, reason);
    exit(EXIT_FAILURE);
}

/**
 * @brief Fail because a required JSON field is absent.
 */
static void config_fail_missing(const char *family, int json_index,
                                const module_config_t *cfg,
                                const char *field)
{
    char reason[96];

    snprintf(reason, sizeof(reason), "missing required field \"%s\"", field);
    config_load_fail(family, json_index, cfg, reason);
}

/**
 * @brief Fail because a string field is present but empty.
 */
static void config_fail_empty(const char *family, int json_index,
                              const module_config_t *cfg,
                              const char *field)
{
    char reason[96];

    snprintf(reason, sizeof(reason), "empty value for \"%s\"", field);
    config_load_fail(family, json_index, cfg, reason);
}

/**
 * @brief Fail because a numeric field is outside the allowed range.
 */
static void config_fail_range(const char *family, int json_index,
                              const module_config_t *cfg,
                              const char *field, const char *range)
{
    char reason[96];

    snprintf(reason, sizeof(reason), "\"%s\" out of range (%s)", field, range);
    config_load_fail(family, json_index, cfg, reason);
}

/**
 * @brief Fail because a field value is not allowed.
 */
static void config_fail_invalid(const char *family, int json_index,
                                const module_config_t *cfg,
                                const char *field, const char *expected)
{
    char reason[128];

    snprintf(reason, sizeof(reason), "invalid \"%s\" (expected %s)",
             field, expected);
    config_load_fail(family, json_index, cfg, reason);
}

/**
 * @brief Validate one enabled module entry after parsing.
 * @param family Config section name (INVERTER or UPS).
 * @param json_index Index in the JSON array.
 * @param cfg Parsed module config.
 */
static void validate_module_config(const char *family, int json_index,
                                   json_object *json_obj,
                                   const module_config_t *cfg)
{
    if (!json_has_field(json_obj, "name")) {
        config_fail_missing(family, json_index, cfg, "name");
    }
    if (str_is_empty(cfg->name)) {
        config_fail_empty(family, json_index, cfg, "name");
    }

    if (!json_has_field(json_obj, "modbus_uid")) {
        config_fail_missing(family, json_index, cfg, "modbus_uid");
    }
    if (cfg->modbus_uid < MODBUS_UID_MIN || cfg->modbus_uid > MODBUS_UID_MAX) {
        config_fail_range(family, json_index, cfg, "modbus_uid", "1-247");
    }

    if (cfg->format == MODBUS_FORMAT_TCP) {
        if (!json_has_field(json_obj, "ip")) {
            config_fail_missing(family, json_index, cfg, "ip");
        }
        if (str_is_empty(cfg->ip)) {
            config_fail_empty(family, json_index, cfg, "ip");
        }
        if (!json_has_field(json_obj, "port")) {
            config_fail_missing(family, json_index, cfg, "port");
        }
        if (cfg->port < 1 || cfg->port > 65535) {
            config_fail_range(family, json_index, cfg, "port", "1-65535");
        }
        return;
    }

    if (!json_has_field(json_obj, "path")) {
        config_fail_missing(family, json_index, cfg, "path");
    }
    if (str_is_empty(cfg->path)) {
        config_fail_empty(family, json_index, cfg, "path");
    }
    if (!json_has_field(json_obj, "baud_rate")) {
        config_fail_missing(family, json_index, cfg, "baud_rate");
    }
    if (!validate_rtu_baud(cfg->baud_rate)) {
        config_fail_invalid(family, json_index, cfg, "baud_rate",
                            "4800-115200, step 100");
    }
}

/**
 * @brief Populate one module_config_t from JSON.
 * @param family Config section name (INVERTER or UPS).
 * @param json_index Index in the JSON array.
 * @param json_obj Source JSON object.
 * @param config Destination config.
 */
static void load_module_config(const char *family, int json_index,
                               json_object *json_obj, module_config_t *config)
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

    config->format = MODBUS_FORMAT_RTU;
    if (json_object_object_get_ex(json_obj, "modbus_format", &field)) {
        if (!parse_modbus_format_value(json_object_get_string(field),
                                       &config->format)) {
            config_fail_invalid(family, json_index, config, "modbus_format",
                                "RTU or TCP");
        }
    }

    /* RTU: path/baud_rate; TCP: ip/port — union storage, parse by format. */
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

}

/**
 * @brief Load enabled Inverter entries from JSON.
 * @param inverter_array INVERTER array object.
 */
static void load_inverter_config(json_object *inverter_array)
{
    if (!inverter_array ||
        !json_object_is_type(inverter_array, json_type_array)) {
        LOG_WARNING("INVERTER configuration not found or invalid.");
        return;
    }

    int valid_count = 0;
    int array_length = json_object_array_length(inverter_array);

    for (int i = 0; i < array_length; i++) {
        json_object *inv_obj = json_object_array_get_idx(inverter_array, i);
        json_object *enabled_obj = NULL;

        if (!json_object_object_get_ex(inv_obj, "enabled", &enabled_obj) ||
            !json_object_get_boolean(enabled_obj)) {
            continue;
        }

        load_module_config("INVERTER", i, inv_obj,
                           &global_config.inverter[valid_count]);
        validate_module_config("INVERTER", i, inv_obj,
                               &global_config.inverter[valid_count]);
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
 * @brief Load enabled UPS entries from JSON.
 * @param ups_array UPS array object.
 */
static void load_ups_config(json_object *ups_array)
{
    if (!ups_array || !json_object_is_type(ups_array, json_type_array)) {
        LOG_WARNING("UPS configuration not found or invalid.");
        return;
    }

    int valid_count = 0;
    int array_length = json_object_array_length(ups_array);

    for (int i = 0; i < array_length; i++) {
        json_object *ups_obj = json_object_array_get_idx(ups_array, i);
        json_object *enabled_obj = NULL;

        if (!json_object_object_get_ex(ups_obj, "enabled", &enabled_obj) ||
            !json_object_get_boolean(enabled_obj)) {
            continue;
        }

        load_module_config("UPS", i, ups_obj,
                           &global_config.ups[valid_count]);
        validate_module_config("UPS", i, ups_obj,
                               &global_config.ups[valid_count]);
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
