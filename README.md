# WetOrDead

Battery-powered Zigbee soil-moisture sensor built on the
[Seeed XIAO ESP32C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#specifications).
It shows up in Zigbee2MQTT / ZHA as an analog *Humidity* input (0–100 %) with a battery level.

## How it works

The device spends almost all of its time in deep sleep. Every wake (30 min by default):

1. **Sample the probe.** It powers the probe from `D1`, waits 60 ms in light sleep, averages the reading on `A0`,
   then powers the probe off.
2. **Decide.** A report is due if nothing has been reported yet, if moisture has moved by at least 5 percentage points
   since the last *acknowledged* report, or if 12 wakes (6 h) have passed. If no report is due, it goes back to sleep
   without ever starting the radio.
3. **Report.** It reads the battery on `A2` (through a 1:2 divider), joins or rejoins the network, sends the moisture
   and battery reports, and waits up to 2 s for the coordinator to acknowledge them.
4. **Sleep.** If the report failed, it is retried on every wake, and the sleep interval doubles with each consecutive
   failure (capped at 6 h). After 20 consecutive failures the stored network is wiped and the device pairs again.

Home Assistant only records a value when it changes. So if a reading (moisture rounded to 0.1 %, battery to 1 %) is
the same as the value sent last time, it is moved by one step before sending. Every report, heartbeats included, then
shows up as a change.

After a power-on or a press of the reset button, the device stays awake for 10 s so new firmware can be uploaded over
USB.

## Building

| PlatformIO env | Purpose                                                    |
| -------------- | ---------------------------------------------------------- |
| `main`         | Production firmware                                        |
| `main-debug`   | Same, with serial logging (`-DDEBUG_SERIAL=true`)          |
| `ZigbeeMicro`  | Minimal Zigbee heartbeat test device (`tests/ZigbeeMicro`) |

Add `-DDEBUG_MODE=true` to the build flags for bench testing: 10-minute cycles, a heartbeat every 3 wakes, and the
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
