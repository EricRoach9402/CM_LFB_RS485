# CM_LFB_RS485

**Linux Modbus gateway** — Inverter (RTU/TCP) + UPS (RTU/TCP) → shared pool → CMOS / Alarm

```
x86/cm_LFB   or   arm/cm_LFB
```

---

## Environment

### First-time setup

```bash
make setup ARCH=x86    # gcc, libjson-c-dev
make setup ARCH=arm    # gcc-aarch64-linux-gnu, curl
```

```bash
sudo apt install gcc libjson-c-dev
sudo apt install gcc-aarch64-linux-gnu curl   # ARM only
```

### Build

```bash
make ARCH=x86              # → x86/cm_LFB
make ARCH=arm              # → arm/cm_LFB  (json-c auto-fetched to build/deps/ on first run)
make ARCH=x86 DEBUG=DEBUG
make clean && make ARCH=x86 rebuild
```

### Run

```bash
./x86/cm_LFB -c ./config/config.json -l 2
# -c  config path (default ./config/config.json)
# -l  0=VERBOSE 1=DEBUG 2=INFO 3=WARN 4=ERROR
```

### Deploy

```bash
ldd x86/cm_LFB    # json-c linked statically by default; needs libc / libpthread / libdl only
```

```bash
make ARCH=x86 JSONC_LINK_MODE=dynamic    # dynamic link (target host needs libjson-c.so)
make ARCH=arm JSONC_ARM_LIB=... JSONC_ARM_INC=...   # custom SDK
```

### Runtime dependencies

| Item | Value |
|------|-------|
| CMOS master | `127.0.0.1:10000` |
| Inverter publish | topic `inverter`, port `13000` |
| UPS publish | topic `ups`, port `12000` |
| Alarm DB | `/home/cm/LFB_Feeder_Kit/Logs/{inverter,ups}_alarm/` |

---

## Core features

```
config.json
    ↓
┌─────────────────┐   RS485 (bus_coord) / TCP   ┌──────────┐
│ Inverter module │ ←──────────────────────────→ │  Device  │
│ UPS module      │ ←──────────────────────────→ │  (Modbus)│
└────────┬────────┘                             └──────────┘
         │ read/write
         ▼
   internal_pool[]  ←── devices/*/*_map.c
         │
    ┌────┴────┐
    ▼         ▼
 CMOS pub   alarm_bridge → SQLite
 (child)     (parent)
```

| Module | Transport | Notes |
|--------|-----------|-------|
| **Inverter** | RTU / TCP | FC03 poll, FC06/16 writes, HMI command queue |
| **UPS** | RTU / TCP | FC03 poll (profile is all RO) |
| **CMOS bridge** | IPC | parent: subscriber; child: publisher (fork) |
| **Alarm** | pool watch | `ALARM_COND_CHANGE` on selected registers → SQLite |

RTU units on the same serial `path` share a `bus_coord` mutex. TCP units use a dedicated socket per entry (no bus coordination).

> **Inverter RTU vs TCP on connect:** RTU runs `init_inverter_reg()` (command source, baud, limits, stop, etc.). TCP connect only opens the socket — no register programming. If TCP control commands have no effect on site, check the inverter manual for Ethernet command-source settings.

### config example

RTU:

```json
{
    "INVERTER": [{
        "name": "Inverter#1", "enabled": true,
        "modbus_format": "RTU", "modbus_uid": 1,
        "path": "/dev/ttyUSB0", "baud_rate": 9600
    }],
    "UPS": [{
        "name": "UPS#1", "enabled": true,
        "modbus_format": "RTU", "modbus_uid": 2,
        "path": "/dev/ttyUSB0", "baud_rate": 9600
    }]
}
```

TCP (Inverter or UPS — `ip` and `port` are both required; there is no default port):

```json
{
    "INVERTER": [{
        "name": "Inverter#1", "enabled": true,
        "modbus_format": "TCP", "modbus_uid": 1,
        "ip": "192.168.1.10", "port": 502
    }]
}
```

### config fields

| Field | Used by daemon | Notes |
|-------|----------------|-------|
| `name`, `enabled`, `modbus_uid` | yes | Required for enabled entries (see validation below) |
| `modbus_format` | yes | `"RTU"` or `"TCP"`; omitted → RTU |
| `path`, `baud_rate` | yes (RTU) | Both required for RTU |
| `ip`, `port` | yes (TCP) | Both required for TCP |
| `modbus_role`, `gpio` | ignored | Accepted in JSON for compatibility; not loaded |

Poll cycle interval (200 ms between full scan rounds) is `MODBUS_DEFAULT_POLL_CYCLE_INTERVAL_MS` in `include/modbus_defaults.h`.

`enabled: false` entries are **skipped when config is loaded** — they are not kept in `global_config`. Toggling a unit requires editing `config.json` and restarting the daemon.

### Load-time validation

Each **enabled** INVERTER / UPS entry is validated in `config_loader.c` before any module thread starts. On failure the daemon logs `[config] ...` and exits immediately (same severity as a missing JSON file).

| Check | RTU | TCP |
|-------|-----|-----|
| `name` | required, non-empty | required, non-empty |
| `modbus_uid` | required, 1–247 | required, 1–247 |
| `path` | required, non-empty | — |
| `baud_rate` | required, 4800–115200 (step 100) | — |
| `ip` | — | required, non-empty |
| `port` | — | required, 1–65535 |
| `modbus_format` | if present: `RTU` or `TCP` only | same |

Error message format:

| Situation | Message |
|-----------|---------|
| JSON key missing | `missing required field "field"` |
| String present but empty | `empty value for "field"` |
| Numeric out of range | `"field" out of range (min-max)` |
| Value not allowed | `invalid "field" (expected ...)` |

Example: `[config] INVERTER[0] 'Inverter#1': missing required field "port"`

---

## Design

### config vs register map

| Task | Where |
|------|-------|
| Rewire (UID / serial / IP / port / baud) | `config/config.json` + restart |
| Enable / disable a unit | `config/config.json` + restart (`enabled: false` omitted at load) |
| Change register layout or CMOS keys | `devices/<dev>/*_map.c` (pool_address, description) |

Current built-in profiles:

| Device | profile | pool base |
|--------|---------|-----------|
| Inverter | `inverter1_profile` | `0xA000` |
| UPS | `ups1_profile` | `0xA200` |

HMI write/init commands are routed to the unit registered at startup (`inverter_get_primary_uid()` / `ups_get_primary_uid()`).

### Init flag (operator-triggered)

```
HMI init cmd → bridge → inverter_init_request() / ups_init_request()
                              ↓
                    run_init_sequence()  →  int_*_init_flag_reg = 1
                              ↓
                    CMOS publish "initial_flag"
```

Connect success does **not** set init flag automatically.

### CMOS fork rule

```c
// parent must not call cmos_publish() — write internal_pool[]; child publisher reads it
```

---

## Maintenance

| ⚠️ | Note |
|----|------|
| **CMOS key = map description** | renaming `description` in `*_map.c` changes the HMI subscription key |
| **table sort order** | `device_address` must be ascending (binary search) |
| **pool_address** | absolute values in `*_map.c` (e.g. `0xA000`, `0xA200`); not derived at runtime |
| **shared RS485** | same `path` → `bus_coord` mutex (max **8** distinct paths); Modbus UIDs must be unique on a bus; TCP entries are independent |
| **Alarm paths** | hardcoded in `*_alarm.c`, not configurable |
| **Dual alarm systems** | internal SQLite (`alarm_bridge`) vs external `config/alarm_user_config.ini` (CMOS topic watch for HMI); daemon does **not** load the INI |
| **Comm loss marker** | sustained read failure sets all profile pool slots to `0xFFFF` |
| **Startup** | invalid enabled config → exit at load; CMOS pub / module / alarm init failure → cleanup and `EXIT_FAILURE` |
| **lib/** | changes require separate approval |

### Layout

```
src/           modules, bridge, alarm, main
devices/       register maps (hardware contract)
config/        config.json (daemon); alarm_user_config.ini (external HMI alarm node only)
lib/           modbus, cmos, device_map, alarm, sqlite
```

---

## Multi-unit (not supported)

**Current scope: one Inverter + one UPS.** That matches the default `config.json` and the built-in `inverter1_profile` / `ups1_profile` maps.

Do not add a second entry of the same device family to config expecting it to work — the loader and module loops allow multiple entries, but profile binding, CMOS publish/command routing, and alarm reads are still single-profile.

If multi-unit support is needed later:

```
□ pick a new pool base (must not overlap existing profiles)
□ devices/<name>/<name>_map.c   — copy table, update all pool_address values
□ devices/<name>/<name>_map.h   — new profile
□ config.json                   — new entry (uid / path / baud or ip / port)
□ *_module.c                    — bind profile per unit (remove hardcoded *1_profile)
□ *_cmos_bridge.c               — publish table + command routing per unit
□ *_alarm.c                     — point each ctx read_data at the correct profile
```

Same-model 2nd unit vs new model: same steps. If the register layout is identical, copy the table and change the pool base only.
