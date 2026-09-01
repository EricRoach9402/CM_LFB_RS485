# CM_LFB_RS485

**Linux Modbus gateway** — Inverter (RTU) + UPS (RTU/TCP) → shared pool → CMOS / Alarm

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
┌─────────────────┐     RS485 (bus_coord)     ┌──────────┐
│ Inverter module │ ←────────────────────────→ │  Device  │
│ UPS module      │ ←────────────────────────→ │  (Modbus)│
└────────┬────────┘                           └──────────┘
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
| **Inverter** | RTU only | FC03 poll, FC06/16 writes, HMI command queue; no TCP |
| **UPS** | RTU / TCP | FC03 poll (profile is all RO) |
| **CMOS bridge** | IPC | parent: subscriber; child: publisher (fork) |
| **Alarm** | pool watch | `ALARM_COND_CHANGE` on selected registers → SQLite |

### config example

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

```json
"modbus_format": "TCP", "ip": "192.168.1.10", "port": 502
```

> **Inverter: RTU only.** There is no TCP transport path in `inverter_module.c`; a TCP entry is not rejected at load time but will fail at runtime (empty serial `path`, reconnect loop). **UPS** supports RTU and TCP.

### config fields

| Field | Used by daemon | Notes |
|-------|----------------|-------|
| `name`, `enabled`, `modbus_format`, `modbus_uid` | yes | |
| `path`, `baud_rate` | yes (RTU) | Required for RTU; not validated at load time |
| `ip`, `port` | yes (TCP, UPS only) | Required for TCP; not validated at load time |
| `modbus_role`, `gpio` | parsed only | Stored in config; no runtime effect today |
| `rtu_poll_interval_ms` | no | Fixed at 200 ms in `config_loader.c` |

`enabled: false` entries are **skipped when config is loaded** — they are not kept in `global_config`. Toggling a unit requires editing `config.json` and restarting the daemon.

---

## Design

### config vs register map

| Task | Where |
|------|-------|
| Rewire (UID / serial / baud) | `config/config.json` + restart |
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
| **shared RS485** | same `path` → `bus_coord` mutex (max **8** distinct paths); Modbus UIDs must be unique on a bus |
| **Alarm paths** | hardcoded in `*_alarm.c`, not configurable |
| **Dual alarm systems** | internal SQLite (`alarm_bridge`) vs external `config/alarm_user_config.ini` (CMOS topic watch for HMI); daemon does **not** load the INI |
| **Comm loss marker** | sustained read failure sets all profile pool slots to `0xFFFF` |
| **Startup** | module / alarm / CMOS init errors are logged; daemon keeps running |
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
□ config.json                   — new entry (uid / path / baud)
□ *_module.c                    — bind profile per unit (remove hardcoded *1_profile)
□ *_cmos_bridge.c               — publish table + command routing per unit
□ *_alarm.c                     — point each ctx read_data at the correct profile
```

Same-model 2nd unit vs new model: same steps. If the register layout is identical, copy the table and change the pool base only.
