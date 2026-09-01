# CM_LFB_RS485

**Linux Modbus gateway** — Inverter (RTU) + UPS (RTU/TCP) → shared pool → CMOS / Alarm

```
x86/cm_LFB   or   arm/cm_LFB
```

---

## ⚙️ Environment

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

## 🚀 Core features

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
| **Inverter** | RTU only | FC03 poll, FC06/16 writes, HMI command queue |
| **UPS** | RTU / TCP | FC03 poll (profile is all RO) |
| **CMOS bridge** | IPC | parent: subscriber; child: publisher (fork) |
| **Alarm** | pool watch | fault / warning register changes → DB |

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

> Inverter with `TCP` is **ignored**; UPS supports TCP.

---

## 🧠 Design

### config vs table

| Task | Where |
|------|-------|
| Rewire (UID / serial / baud / enabled) | `config/config.json` |
| **Add a unit** (same or new model) | new **pool block** + `devices/<dev>/*_map.c` profile + `config.json` |

```bash
# ❌ config only, same pool → register_profile collision at startup
# ✅ 2nd same-model unit = new model: assign new pool base, register new profile
```

### Default configuration

Built-in **one profile / pool per device family**. Do not add a 2nd unit in config until a new pool is assigned:

| Device | profile | pool base |
|--------|---------|-----------|
| Inverter | `inverter1_profile` | `0xA000` |
| UPS | `ups1_profile` | `0xA200` |

```c
// bridge commands target the first unit actually registered by start_*_modules()
int inverter_get_primary_uid(uint8_t *out_uid);
int ups_get_primary_uid(uint8_t *out_uid);
```

Adding a 2nd unit still needs a new profile + pool, CMOS per-unit routing (bridge currently sends commands to the first registered unit), and alarm ctx updates (still a single profile).

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

## 🔧 Maintenance

| ⚠️ | Note |
|----|------|
| **CMOS key = map description** | renaming `description` in `*_map.c` changes the HMI subscription key |
| **table sort order** | `device_address` must be ascending (binary search) |
| **shared RS485** | same `path` → `bus_coord` mutex; Modbus UIDs must be unique |
| **Alarm paths** | hardcoded in `*_alarm.c`, not configurable |
| **lib/** | changes require separate approval |

### Layout

```
src/           modules, bridge, alarm, main
devices/       register maps (hardware contract)
config/        config.json, alarm_user_config.ini (external HMI)
lib/           modbus, cmos, device_map, alarm, sqlite
```

### Add-a-unit checklist

```
□ pick new pool base (must not overlap existing profiles)
□ devices/<name>/<name>_map.c   — copy table, update all pool_address values
□ devices/<name>/<name>_map.h   — new profile
□ config.json                   — new entry (uid / path / baud)
□ *_module.c                    — bind profile
□ *_cmos_bridge.c               — command routing (currently first registered unit)
□ *_alarm.c                     — alarm ctx (if needed)
```

2nd same-model unit vs new model: **same steps**. If register layout is identical, copy the table and change pool base only.
