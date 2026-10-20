# LODESTAR HELIOS - NO GPS

**ALPHA** firmware by FEAR REACTIVE for the Seeed SenseCAP Solar Node **P1 without GPS**, built on [MeshCore](https://github.com/meshcore-dev/MeshCore).

HELIOS is the LODESTAR PULSAR repeater (uplink scoring and the `lodestar` commands) plus a **read-only battery watch** for solar nodes. It watches the battery voltage, reports GOOD / LOW / CRITICAL, shows the trend (charging, discharging, steady) and keeps 48 hours of hourly min/max. It never changes how the radio or charger behaves.

The GPS driver is compiled out, so a plain P1 doesn't spend flash or power on it. For the P1-Pro use [helios-gps](https://github.com/propertymonitor-web/helios-gps).

## Status: ALPHA
- The logic is unit-tested on a PC and the patch is checked against current MeshCore on every push.
- On hardware, the author is field-testing the P1-Pro build for about a month. Treat results as unproven until then.
- Battery percentages are an approximation from a Li-ion voltage curve (about 4.2 V full, 3.3 V empty), not a fuel gauge.

## Get the firmware
Download `firmware.uf2` from the [Releases](../../releases) page. Double-tap the board's RST button so it shows up as a USB drive, then drag the file onto it. Seeed's guide: https://wiki.seeedstudio.com/get_started_with_meshcore_solar_node/

**Change the default admin password (`password`) first.**

## Commands (repeater CLI)
| Command | What it does |
|---|---|
| `helios` | Battery mV, approximate %, state (GOOD/LOW/CRITICAL) and trend |
| `helios history` | Last 48 hours, one line per hour with min and max mV (kept in RAM, cleared on reboot) |
| `lodestar ...` | Uplink scoring commands from LODESTAR PULSAR |

Defaults: GOOD at 3700 mV and up, LOW below 3500 mV, with 50 mV hysteresis so it doesn't flap. Sampled once a minute. Readings under 2.5 V or over 5 V are ignored as bad reads.

## Build it yourself
Run **Actions > Build ... > Run workflow**. Tick *publish* and enter a NEW tag to create a pre-release. Or locally:

    git clone https://github.com/meshcore-dev/MeshCore.git
    python tools/apply_helios.py --meshcore MeshCore --variant nogps
    cd MeshCore && pio run -e HELIOS_NoGps_repeater

## License
Patches and added code follow MeshCore's license (MIT).
