###############################################################################
# Makefile for loop_charger_inverter unit tests
#
# Usage
# ─────
#  Build and run all tests:   make
#  Build only:                make build
#  Run only (after build):    make run
#  Clean:                     make clean
#
# Each test suite is compiled into a standalone executable under build/.
# The 'run' target executes all suites and prints a combined pass/fail
# summary; the exit code is non-zero if any suite reported a failure.
###############################################################################

# ── Directories ──────────────────────────────────────────────────────────
ROOT    = ..
BUILD   = build

# ── Compiler ─────────────────────────────────────────────────────────────
CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -D_GNU_SOURCE -g -O0
CFLAGS += -I$(ROOT)/include
CFLAGS += -I$(ROOT)/devices
CFLAGS += -I$(ROOT)/lib/alarm
CFLAGS += -I$(ROOT)/lib/cmos
CFLAGS += -I$(ROOT)/lib/device_map
CFLAGS += -I$(ROOT)/lib/log
CFLAGS += -I$(ROOT)/lib/modbus
CFLAGS += -Iunity

LDLIBS  = -lpthread

# ── Unity framework ───────────────────────────────────────────────────────
UNITY_SRC = unity/unity.c

# ── Production sources required by each test suite ────────────────────────
# alarm_engine has no extra dependencies beyond its own .c
SRCS_ALARM_ENGINE    = $(ROOT)/lib/alarm/alarm_engine.c

# device_register_map depends on log and pthreads
SRCS_DEVICE_MAP      = $(ROOT)/lib/device_map/device_register_map.c \
                       $(ROOT)/lib/log/log.c

# inverter_map depends on device_register_map (and log transitively)
SRCS_INVERTER_MAP    = $(ROOT)/devices/inverter/inverter_map.c \
                       $(ROOT)/lib/device_map/device_register_map.c \
                       $(ROOT)/lib/log/log.c

# modbus_rtu_client needs termios (part of libc, no extra -l needed)
SRCS_MODBUS_RTU      = $(ROOT)/lib/modbus/modbus_rtu_client.c

# ── Test executables ──────────────────────────────────────────────────────
TEST_ALARM_ENGINE    = $(BUILD)/test_alarm_engine
TEST_DEVICE_MAP      = $(BUILD)/test_device_register_map
TEST_INVERTER_MAP    = $(BUILD)/test_inverter_map
TEST_MODBUS_CRC      = $(BUILD)/test_modbus_rtu_crc

ALL_TESTS = \
    $(TEST_ALARM_ENGINE) \
    $(TEST_DEVICE_MAP) \
    $(TEST_INVERTER_MAP) \
    $(TEST_MODBUS_CRC)

# ── Phony targets ─────────────────────────────────────────────────────────
.PHONY: all build run clean

all: build run

build: $(ALL_TESTS)

# ── Link rules ────────────────────────────────────────────────────────────

$(TEST_ALARM_ENGINE): test_alarm_engine.c $(UNITY_SRC) $(SRCS_ALARM_ENGINE)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(TEST_DEVICE_MAP): test_device_register_map.c $(UNITY_SRC) $(SRCS_DEVICE_MAP)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(TEST_INVERTER_MAP): test_inverter_map.c $(UNITY_SRC) $(SRCS_INVERTER_MAP)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(TEST_MODBUS_CRC): test_modbus_rtu_crc.c $(UNITY_SRC) $(SRCS_MODBUS_RTU)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

# ── Run all tests and report summary ─────────────────────────────────────
run:
	@echo ""
	@echo "══════════════════════════════════════════════════════"
	@echo "  Running unit tests for loop_charger_inverter"
	@echo "══════════════════════════════════════════════════════"
	@PASS=0; FAIL=0; \
	for t in $(ALL_TESTS); do \
	    echo ""; \
	    echo "[ $$(basename $$t) ]"; \
	    if $$t; then \
	        PASS=$$((PASS + 1)); \
	    else \
	        FAIL=$$((FAIL + 1)); \
	    fi; \
	done; \
	echo ""; \
	echo "══════════════════════════════════════════════════════"; \
	echo "  Results: $$PASS suite(s) passed, $$FAIL suite(s) failed"; \
	echo "══════════════════════════════════════════════════════"; \
	echo ""; \
	test $$FAIL -eq 0

# ── Clean ─────────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD)
