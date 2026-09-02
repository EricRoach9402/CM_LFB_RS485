/**
 * @file main.c
 * @brief CM LFB RS485 daemon entry point.
 */

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "alarm_bridge.h"
#include "alarm_manager.h"
#include "bus_coord.h"
#include "config_loader.h"
#include "device_register_map.h"
#include "inverter_alarm.h"
#include "inverter_cmos_bridge.h"
#include "inverter_module.h"
#include "log.h"
#include "ups_alarm.h"
#include "ups_cmos_bridge.h"
#include "ups_module.h"

extern system_config_t global_config;

static volatile sig_atomic_t g_running = 1;

static pid_t g_inverter_pub_pid = -1;
static pid_t g_ups_pub_pid = -1;

static void parse_arguments(int argc, char **argv,
                            char **config_path, log_level_t *log_level);
static void setup_signal_handlers(void);
static void signal_handler(int sig);
static bool family_has_enabled(const module_config_t *units, int count);
static pid_t fork_cmos_publisher(const char *label, void (*pub_run)(void));
static void stop_cmos_publisher(pid_t *pid, const char *label);
static void shutdown_runtime(void);
static int startup_failed(const char *reason);

/**
 * @brief Daemon entry point.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status.
 */
int main(int argc, char **argv)
{
    char *config_path = "./config/config.json";
    log_level_t log_level = LOG_LEVEL_INFO;

    LOG_INFO("***************************************");
    LOG_INFO(" CM LFB_RS485  version: %s",
             CM_LFB_RS485_VERSION);
    LOG_INFO("***************************************");

    parse_arguments(argc, argv, &config_path, &log_level);
    set_log_level(log_level);

    LOG_INFO("Loading configuration: %s", config_path);
    load_json_config(config_path);

    bus_coord_init();

    /* Shared pool must exist before forking publisher children. */
    device_register_map_init();

    setup_signal_handlers();

    if (family_has_enabled(global_config.inverter,
                           global_config.inverter_count)) {
        g_inverter_pub_pid = fork_cmos_publisher("Inverter",
                                                 inverter_cmos_pub_run);
        if (g_inverter_pub_pid < 0) {
            return startup_failed("Inverter CMOS publisher failed to start.");
        }
    }

    if (family_has_enabled(global_config.ups, global_config.ups_count)) {
        g_ups_pub_pid = fork_cmos_publisher("UPS", ups_cmos_pub_run);
        if (g_ups_pub_pid < 0) {
            return startup_failed("UPS CMOS publisher failed to start.");
        }
    }

    if (start_inverter_modules(global_config.inverter,
                               global_config.inverter_count) != 0) {
        return startup_failed("One or more Inverter modules failed to start.");
    }

    if (start_ups_modules(global_config.ups,
                          global_config.ups_count) != 0) {
        return startup_failed("One or more UPS modules failed to start.");
    }

    inverter_alarm_register_all();
    ups_alarm_register_all();

    if (alarm_bridge_start() != 0) {
        return startup_failed("Alarm bridge failed to start.");
    }

    LOG_INFO("All modules started. Waiting for shutdown signal …");

    while (g_running) {
        sleep(1);
    }

    shutdown_runtime();

    LOG_INFO("Shutdown complete.");
    return 0;
}

/**
 * @brief Handle shutdown signals.
 * @param sig Signal number.
 */
static void signal_handler(int sig)
{
    LOG_INFO("Received signal %d. Shutting down …", sig);
    g_running = 0;
}

/**
 * @brief Install SIGINT and SIGTERM handlers.
 */
static void setup_signal_handlers(void)
{
    struct sigaction sa = {
        .sa_handler = signal_handler,
        .sa_flags = 0,
    };
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

/**
 * @brief Parse command-line options.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @param config_path Output config file path.
 * @param log_level Output log level.
 */
static void parse_arguments(int argc, char **argv,
                            char **config_path, log_level_t *log_level)
{
    int opt;

    while ((opt = getopt(argc, argv, "c:l:h")) != -1) {
        switch (opt) {
        case 'c':
            *config_path = optarg;
            break;
        case 'l':
            *log_level = (log_level_t)atoi(optarg);
            break;
        case 'h':
        default:
            fprintf(stderr,
                    "Usage: %s [-c config.json] [-l log_level]\n"
                    "  -c  Path to JSON configuration file\n"
                    "  -l  Log level: 0=VERBOSE 1=DEBUG 2=INFO 3=WARN 4=ERROR\n",
                    argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}

/**
 * @brief Return true if any unit in the array is enabled.
 * @param units Module config array.
 * @param count Array length.
 * @return true if at least one unit is enabled.
 */
static bool family_has_enabled(const module_config_t *units, int count)
{
    for (int i = 0; i < count; i++) {
        if (units[i].enabled) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Fork a CMOS publisher child process.
 * @param label Process label for logs.
 * @param pub_run Publisher entry function.
 * @return Child pid, or -1 on failure.
 */
static pid_t fork_cmos_publisher(const char *label, void (*pub_run)(void))
{
    pid_t pid = fork();
    if (pid < 0) {
        LOG_ERROR("fork(%s CMOS pub) failed: %s", label, strerror(errno));
        return -1;
    }

    if (pid == 0) {
        pub_run();
        _exit(0);
    }

    LOG_INFO("Forked %s CMOS publisher child pid=%d.", label, (int)pid);
    return pid;
}

/**
 * @brief Stop a CMOS publisher child process.
 * @param pid Publisher pid pointer.
 * @param label Process label for logs.
 */
static void stop_cmos_publisher(pid_t *pid, const char *label)
{
    if (!pid || *pid <= 0) {
        return;
    }

    if (kill(*pid, SIGTERM) != 0 && errno != ESRCH) {
        LOG_WARNING("kill(%s CMOS pub pid=%d) failed: %s",
                    label, (int)*pid, strerror(errno));
    }

    int status = 0;
    if (waitpid(*pid, &status, 0) < 0 && errno != ECHILD) {
        LOG_WARNING("waitpid(%s CMOS pub pid=%d) failed: %s",
                    label, (int)*pid, strerror(errno));
    } else {
        LOG_INFO("%s CMOS publisher child pid=%d exited.", label, (int)*pid);
    }

    *pid = -1;
}

/**
 * @brief Stop modules, alarm bridge, and CMOS publisher children.
 */
static void shutdown_runtime(void)
{
    alarm_bridge_stop();
    stop_inverter_modules();
    stop_ups_modules();
    alarm_manager_close();
    stop_cmos_publisher(&g_inverter_pub_pid, "Inverter");
    stop_cmos_publisher(&g_ups_pub_pid, "UPS");
}

/**
 * @brief Log startup failure, tear down partial runtime, and exit non-zero.
 * @param reason Human-readable failure reason.
 * @return EXIT_FAILURE for main().
 */
static int startup_failed(const char *reason)
{
    LOG_ERROR("%s", reason);
    shutdown_runtime();
    LOG_INFO("Startup aborted.");
    return EXIT_FAILURE;
}
