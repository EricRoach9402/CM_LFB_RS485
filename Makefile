###############################################################################
# Makefile for loop_charger_inverter
#
# Supports cross-compilation for x86 and ARM platforms.
# Build output is placed in x86/ or arm/ respectively.
#
# Usage
# ─────
# 0. First time on a fresh machine, install system dependencies:
#      make setup ARCH=x86   (installs gcc, libjson-c-dev)
#      make setup ARCH=arm   (installs gcc-aarch64-linux-gnu, curl)
#      (requires sudo; you will be prompted for a password)
#
# 1. Build for x86 (host):
#      make ARCH=x86
#
# 2. Build for ARM (requires aarch64-linux-gnu-gcc on PATH):
#      make ARCH=arm
#      The required json-c arm64 package is downloaded automatically from
#      ports.ubuntu.com on first build and cached in build/deps/ (gitignored).
#      Uses curl if available, falls back to wget otherwise.
#
# 3. Debug build:
#      make ARCH=x86 DEBUG=DEBUG
#
# 4. Clean all output:
#      make clean
#
# 5. Rebuild from scratch:
#      make ARCH=<x86|arm> rebuild
#
# Dependencies
# ────────────
# x86:
#   sudo apt install gcc libjson-c-dev
#
# ARM cross-compilation:
#   sudo apt install gcc-aarch64-linux-gnu curl
#   (json-c arm64 is fetched automatically on first build; no system changes needed)
#
# Custom SDK / sysroot
# ────────────────────
# Override JSONC_ARM_LIB and JSONC_ARM_INC to point at your SDK instead of the
# auto-downloaded package:
#   make ARCH=arm JSONC_ARM_LIB=/opt/sdk/lib/libjson-c.a \
#                 JSONC_ARM_INC=/opt/sdk/include
#
# For x86 with a custom pkg-config wrapper:
#   make ARCH=x86 PKG_CONFIG=/opt/sdk/bin/pkg-config
#
# x86 linking mode for json-c
###############################################################################

# ── Build options ─────────────────────────────────────────────────────────
ARCH  ?= x86
DEBUG ?= RELEASE

ARCH  := $(strip $(ARCH))
DEBUG := $(strip $(DEBUG))

# ── Target binary name ────────────────────────────────────────────────────
TARGET = cm_LFB

# ── Source files ──────────────────────────────────────────────────────────
# Project-specific sources
SRCS = \
	src/main.c \
	src/config_loader.c \
	src/bus_coord.c \
	src/inverter_module.c \
	src/inverter_cmos_bridge.c \
	src/inverter_alarm.c \
	src/alarm_manager.c \
	devices/inverter/inverter_map.c \
	src/ups_module.c \
	src/ups_cmos_bridge.c \
	src/ups_alarm.c \
	devices/ups/ups_map.c

# Generic library sources (portable across projects)
SRCS += \
	lib/alarm/alarm_engine.c \
	lib/alarm/alarm_bridge.c \
	lib/cmos/cmos_pub.c \
	lib/cmos/cmos_sub.c \
	lib/modbus/modbus_rtu_client.c \
	lib/modbus/modbus_tcp.c \
	lib/modbus/modbus_tcp_client.c \
	lib/device_map/device_register_map.c \
	lib/log/log.c \
	lib/sqlite3/sqlite3.c

OBJS = $(SRCS:.c=.o)

# ── ARM toolchain ─────────────────────────────────────────────────────────
ARM_CC = aarch64-linux-gnu-gcc

# ── json-c: ARM (auto-downloaded from ports.ubuntu.com, cached, not in git) ──
#
# Override JSONC_ARM_VERSION to pin a different release, or override
# JSONC_ARM_LIB / JSONC_ARM_INC entirely to use a custom SDK.
JSONC_ARM_VERSION ?= 0.13.1+dfsg-7ubuntu0.3
JSONC_ARM_DEB      = libjson-c-dev_$(JSONC_ARM_VERSION)_arm64.deb
JSONC_ARM_URL      = http://ports.ubuntu.com/ubuntu-ports/pool/main/j/json-c/$(JSONC_ARM_DEB)
JSONC_ARM_DIR     ?= build/deps/arm64
JSONC_ARM_LIB     ?= $(JSONC_ARM_DIR)/usr/lib/aarch64-linux-gnu/libjson-c.a
JSONC_ARM_INC     ?= $(JSONC_ARM_DIR)/usr/include

_GOALS             := $(if $(MAKECMDGOALS),$(MAKECMDGOALS),all)
_NON_BUILD_GOALS   := $(filter clean setup,$(_GOALS))
_BUILD_GOALS       := $(filter-out clean setup,$(_GOALS))
SKIP_TOOLCHAIN_CHECK := $(and $(_NON_BUILD_GOALS),$(if $(_BUILD_GOALS),,1))

# ── Architecture-specific settings ────────────────────────────────────────
ifeq ($(ARCH), x86)
    CC         = gcc
    OUTPUT_DIR = x86
    EXTRA_DEPS =

    # Resolve json-c via system pkg-config.
    PKG_CONFIG      ?= pkg-config
    JSONC_CFLAGS    := $(shell $(PKG_CONFIG) --cflags json-c 2>/dev/null)
    JSONC_LIBS_RAW  := $(shell $(PKG_CONFIG) --libs   json-c 2>/dev/null)

    # JSONC_LINK_MODE = static (default) | dynamic
    #   static  -> libjson-c.a is linked directly into the binary, so it
    #              runs on any x86 host without libjson-c installed.
    #   dynamic -> smaller binary, but the target host must have
    #              libjson-c.so.* installed at runtime.
    JSONC_LINK_MODE ?= static
    ifeq ($(JSONC_LINK_MODE), static)
        JSONC_LIBS := -Wl,-Bstatic $(JSONC_LIBS_RAW) -Wl,-Bdynamic
    else ifeq ($(JSONC_LINK_MODE), dynamic)
        JSONC_LIBS := $(JSONC_LIBS_RAW)
    else
        $(error Unsupported JSONC_LINK_MODE "$(JSONC_LINK_MODE)". Use static or dynamic.)
    endif

    ifeq ($(SKIP_TOOLCHAIN_CHECK),)
        ifeq ($(JSONC_LIBS_RAW),)
            $(error json-c not found. Run: sudo apt install libjson-c-dev (or: make setup ARCH=x86))
        endif
        ifeq ($(JSONC_LINK_MODE), static)
            ifeq ($(shell gcc -print-file-name=libjson-c.a 2>/dev/null),libjson-c.a)
                $(error libjson-c.a (static library) not found. It ships inside libjson-c-dev on most distros; run: sudo apt install libjson-c-dev (or build with JSONC_LINK_MODE=dynamic))
            endif
        endif
    endif

else ifeq ($(ARCH), arm)
    CC         = $(ARM_CC)
    OUTPUT_DIR = arm
    # EXTRA_DEPS triggers the download rule before the link step.
    EXTRA_DEPS   = $(JSONC_ARM_LIB)
    JSONC_CFLAGS = -I$(JSONC_ARM_INC)
    # Link the static .a directly; produces a self-contained ARM binary.
    JSONC_LIBS   = $(JSONC_ARM_LIB)

    # Preflight: fail fast with a clear message instead of a cryptic
    # "command not found" buried inside make's implicit compile rule.
    ifeq ($(SKIP_TOOLCHAIN_CHECK),)
        ifeq ($(shell command -v $(ARM_CC) 2>/dev/null),)
            $(error $(ARM_CC) not found. Run: sudo apt install gcc-aarch64-linux-gnu (or: make setup ARCH=arm))
        endif
    endif

else
    $(error Unsupported ARCH "$(ARCH)". Use ARCH=x86 or ARCH=arm.)
endif

# ── Common flags ──────────────────────────────────────────────────────────
CFLAGS_COMMON  = -Wall -Wextra -std=c11 -D_GNU_SOURCE
CFLAGS_COMMON += -Iinclude -Idevices
CFLAGS_COMMON += -Ilib/alarm -Ilib/cmos -Ilib/modbus -Ilib/device_map -Ilib/log -Ilib/sqlite3
CFLAGS_COMMON += $(JSONC_CFLAGS)

ifeq ($(DEBUG), DEBUG)
    CFLAGS_COMMON += -g -O0 -DDEBUG_MODE
else
    CFLAGS_COMMON += -O2
endif

CFLAGS  = $(CFLAGS_COMMON)
LDFLAGS =
LDLIBS  = -lpthread -ldl $(JSONC_LIBS)

# ── Build rules ───────────────────────────────────────────────────────────
.PHONY: all clean rebuild setup

all: $(OUTPUT_DIR)/$(TARGET)

# Link: EXTRA_DEPS ensures json-c arm64 is present before linking when ARCH=arm.
# Object files come BEFORE library flags so the linker resolves symbols correctly.
$(OUTPUT_DIR)/$(TARGET): $(EXTRA_DEPS) $(OBJS)
	@mkdir -p $(OUTPUT_DIR)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# ── ARM json-c: download and extract from ports.ubuntu.com ────────────────
#
# Runs only when $(JSONC_ARM_LIB) does not exist (i.e. first build or after
# "make clean"). The downloaded .deb is removed after extraction.
#
# Tries curl first, falls back to wget, since minimal/fresh Ubuntu installs
# commonly ship with only one of the two (or neither).
$(JSONC_ARM_LIB):
	@echo ">>> json-c arm64 not found. Downloading $(JSONC_ARM_VERSION) from ports.ubuntu.com ..."
	@mkdir -p build/deps
	@if command -v curl >/dev/null 2>&1; then \
	    curl -fsSL --retry 3 -o build/deps/$(JSONC_ARM_DEB) $(JSONC_ARM_URL); \
	elif command -v wget >/dev/null 2>&1; then \
	    wget -q --tries=3 -O build/deps/$(JSONC_ARM_DEB) $(JSONC_ARM_URL); \
	else \
	    echo "ERROR: neither curl nor wget found. Run: sudo apt install curl (or: make setup ARCH=arm)"; \
	    exit 1; \
	fi || { echo "ERROR: download failed: $(JSONC_ARM_URL)"; rm -f build/deps/$(JSONC_ARM_DEB); exit 1; }
	@dpkg-deb --extract build/deps/$(JSONC_ARM_DEB) $(JSONC_ARM_DIR) \
	    || { rm -rf $(JSONC_ARM_DIR); exit 1; }
	@rm -f build/deps/$(JSONC_ARM_DEB)
	@echo ">>> json-c arm64 ready."

clean:
	rm -f $(OBJS)
	rm -rf x86 arm

rebuild: clean all

# ── One-time host setup ──────────────────────────────────────────────────
#
# Installs the system packages needed to build on a fresh machine. Requires
# sudo (you will be prompted for a password); this cannot be automated away
# since it modifies system package state.
#   make setup ARCH=x86   -> installs gcc, libjson-c-dev
#   make setup ARCH=arm   -> installs gcc-aarch64-linux-gnu, curl
setup:
ifeq ($(ARCH), x86)
	sudo apt-get update
	sudo apt-get install -y gcc libjson-c-dev
else ifeq ($(ARCH), arm)
	sudo apt-get update
	sudo apt-get install -y gcc-aarch64-linux-gnu curl
else
	$(error Unsupported ARCH "$(ARCH)". Use ARCH=x86 or ARCH=arm.)
endif
