# WetOrDead

Battery-powered Zigbee soil-moisture sensor built on the
[Seeed XIAO ESP32C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#specifications).
It shows up in Zigbee2MQTT / ZHA as an analog *Humidity* input (0–100 %) with a battery level.

## How it works

The device spends almost all of its time in deep sleep. Every wake (30 min by default):

1. **Sample the probe.** It powers the probe from `D1`, waits 60 ms in light sleep, averages the reading on `A0`,
   then powers the probe off.
2. **Decide.** A report is due if nothing has been reported yet, if moisture has moved by at least 5 percentage points
   since the last *acknowledged* report, or if 6 wakes (3 h) have passed. If no report is due, it goes back to sleep
   without ever starting the radio.
3. **Report.** It reads the battery on `A2` (through a 1:2 divider), rejoins the network, and sends the moisture and
   battery reports **straight to the coordinator** (address `0x0000`, endpoint 1). It does not use the device's
   binding table. Any report the coordinator hasn't acknowledged within 1.5 s is re-sent, up to 3 attempts.
4. **Sleep.** If the report failed, it is retried on every wake, and the sleep interval doubles with each consecutive
   failure (capped at 6 h). After 20 consecutive failures the stored network is wiped and the device pairs again.

As a safety net, a cycle that is still awake after 3 minutes (e.g. a hung Zigbee stack) is counted as a failure and
the device deep-sleeps anyway, so a fault can't drain the battery.

The 3 h heartbeat keeps the device well inside ZHA's default 6 h timeout for marking battery devices unavailable.

Home Assistant only records a value when it changes. So if a reading (moisture rounded to 0.1 %, battery to 1 %) is
the same as the value sent last time, it is moved by one step before sending. Every report, heartbeats included, then
shows up as a change.

## Pairing and maintenance

After a power-on or a press of the reset button, the device stays awake:

- **10 s** before measuring, so new firmware can be uploaded over USB.
- **60 s** after joining, so the coordinator can talk to it. ZHA needs this time to interview a newly paired device,
  and it ignores reports from a device it hasn't interviewed.

The 60 s window also applies after the device re-pairs itself following 20 failures. To re-run ZHA's setup on a device
that is already paired, press reset, then click *Reconfigure* on the device page in Home Assistant within that minute.

## Building

| PlatformIO env | Purpose                                                    |
| -------------- | ---------------------------------------------------------- |
| `main`         | Production firmware                                        |
| `main-debug`   | Serial logging (`-DDEBUG_SERIAL=true`), never sleeps       |
| `ZigbeeMicro`  | Minimal Zigbee heartbeat test device (`tests/ZigbeeMicro`) |

The USB serial port disappears in light and deep sleep, so `main-debug` never sleeps:

- the probe warm-up is a plain delay
- between cycles it waits awake instead of deep-sleeping
- Zigbee is started once and stays connected

The logic is the same as production, but the debug build doesn't show real power use or the reboot-and-rejoin path.

Add `-DDEBUG_MODE=true` to the build flags for bench testing: 10-minute cycles, a heartbeat every 3 cycles, and the
user LED lit while the probe warms up.

All tunables (timings, thresholds, pins, calibration) are at the top of [`src/main.cpp`](src/main.cpp).

## Timings

| Phase          | avg | min |  max |
| :------------- | --: | --: | ---: |
| Probe sampling | 100 | 100 |  100 |
| Battery        |   1 |   1 |    1 |
| Zigbee join    | 319 | 316 |  323 |
| Report + ack   | 782 | 353 | 1351 |

- sample size = 5, measured on the previous firmware revision (the phases are unchanged)
- time is in ms
