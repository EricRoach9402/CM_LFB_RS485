# loop_charger_inverter

Inverter monitoring daemon for Linux. Supports x86 and ARM (aarch64) targets.

## Build dependencies

### One-time setup (fresh machine / new computer)

```bash
# x86 (installs gcc, libjson-c-dev)
make setup ARCH=x86

# ARM cross-compilation (installs gcc-aarch64-linux-gnu, curl)
make setup ARCH=arm
```

This runs `sudo apt-get install ...` under the hood, so you'll be prompted
for your sudo password. Equivalent manual commands:

```bash
sudo apt install gcc libjson-c-dev              # x86
sudo apt install gcc-aarch64-linux-gnu curl     # ARM
```

The json-c arm64 library itself is downloaded automatically from
ports.ubuntu.com on the first `make ARCH=arm` run and cached in
`build/deps/` (gitignored) — no manual step needed for that part. The
download uses `curl` if present, otherwise falls back to `wget`.

If you run `make` before `make setup`, the Makefile fails fast with a
clear error telling you exactly what's missing, instead of a cryptic
"command not found".

## Build

```bash
# x86 (default)
make ARCH=x86

# ARM – json-c is fetched automatically on first run
make ARCH=arm

# Debug build
make ARCH=x86 DEBUG=DEBUG

# Clean compiled objects and output binaries
make clean

# Rebuild from scratch
make ARCH=<x86|arm> rebuild
```

## Custom SDK / sysroot

### ARM

Override `JSONC_ARM_LIB` and `JSONC_ARM_INC` to use a pre-built SDK instead
of the auto-downloaded package:

```bash
make ARCH=arm \
     JSONC_ARM_LIB=/opt/my-sdk/lib/libjson-c.a \
     JSONC_ARM_INC=/opt/my-sdk/include
```

### x86

Override `PKG_CONFIG` to point at a custom wrapper:

```bash
make ARCH=x86 PKG_CONFIG=/opt/my-sdk/bin/pkg-config
```

## Runtime dependencies / deploying to another machine

Both `x86/loop_charger_inverter` and `arm/loop_charger_inverter` link json-c
**statically**, so they are self-contained: you can copy the binary to
another machine of the same architecture without installing `libjson-c` (or
even `libjson-c-dev`) there. Verify with `ldd`, which should list only
standard libraries (`libpthread`, `libdl`, `libc`) and no `libjson-c.so.*`.

If you need the old dynamic-linked x86 binary instead (smaller binary, but
the target host must have `libjson-c.so.*` installed), build with:

```bash
make ARCH=x86 JSONC_LINK_MODE=dynamic
```
