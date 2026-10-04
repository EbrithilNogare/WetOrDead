// =============================================================================
// WetOrDead - battery-powered Zigbee soil-moisture sensor (Seeed XIAO ESP32-C6)
//
// The device lives in deep sleep and does all of its work in setup():
//
//   1. Power the probe, let it settle in light sleep, sample it, power it off.
//   2. Decide whether a report is due: nothing reported yet, moisture moved by
//      MOISTURE_REPORT_THRESHOLD since the last acknowledged report, or
//      HEARTBEAT_CYCLES wakes have passed. If not, go straight back to sleep;
//      the radio is never started on those wakes.
//   3. Sample the battery, (re)join the Zigbee network, report moisture and
//      battery, and wait for the coordinator to acknowledge the reports.
//   4. Deep-sleep until the next cycle. Consecutive failures back off
//      exponentially; too many in a row wipe the stored network so the device
//      pairs from scratch.
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <Zigbee.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cmath>

// =============================================================================
// Configuration
// =============================================================================

// Serial logging over USB-CDC. Enabled by the `main-debug` PlatformIO env.
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL false
#endif

// Bench testing: shorter cycles, and the user LED lights while the probe warms up.
#ifndef DEBUG_MODE
#define DEBUG_MODE false
#endif

// --- Wake cycle ---
constexpr uint32_t CYCLE_MS         = (DEBUG_MODE ? 10 : 30) * 60 * 1000;
constexpr uint32_t HEARTBEAT_CYCLES = DEBUG_MODE ? 3 : 12;  // report at least this often, even if unchanged
constexpr uint32_t UPLOAD_WINDOW_MS = 10000;                // stay awake after a manual reset so USB can flash us
constexpr uint32_t MIN_SLEEP_MS     = 1000;

// --- Failure handling ---
constexpr uint32_t BACKOFF_MAX_SHIFT       = 6;                    // sleep doubles per consecutive failure...
constexpr uint32_t BACKOFF_MAX_MS          = 6UL * 60 * 60 * 1000; // ...up to this
constexpr uint32_t NETWORK_RESET_THRESHOLD = 20;                   // consecutive failures before re-pairing

// --- Pins (names from the XIAO_ESP32C6 variant) ---
constexpr uint8_t PIN_PROBE       = A0;
constexpr uint8_t PIN_PROBE_POWER = D1;
constexpr uint8_t PIN_BATTERY     = A2;
constexpr uint8_t PIN_LED         = LED_BUILTIN;  // active-low

// --- Analog front end ---
constexpr uint8_t ADC_SAMPLES = 16;  // averaged per reading

// Soil probe: output voltage at saturation and in dry air, in mV.
constexpr float    PROBE_WET_MV    = 0.0f;
constexpr float    PROBE_DRY_MV    = 3200.0f;
constexpr uint32_t PROBE_WARMUP_MS = 60;
static_assert(PROBE_DRY_MV != PROBE_WET_MV, "probe calibration points must differ");

// Battery: 220k/220k divider; LiPo discharge curve in mV at 0%, 10%, ..., 100%.
constexpr float BATTERY_DIVIDER_RATIO = (220000.0f + 220000.0f) / 220000.0f;
constexpr std::array<float, 11> BATTERY_CURVE_MV = {3200, 3442, 3547, 3673, 3736, 3776, 3812, 3880, 3925, 3953, 4100};

constexpr float MOISTURE_REPORT_THRESHOLD = 5.0f;  // percentage points

// Smallest step of each reported value; also the bump that keeps reports distinct (see distinctFrom()).
constexpr float MOISTURE_STEP = 0.1f;  // matches the advertised Analog Input resolution
constexpr float BATTERY_STEP  = 1.0f;

// --- Zigbee ---
constexpr uint8_t  ZIGBEE_ENDPOINT          = 10;
constexpr int8_t   TX_POWER_DBM             = 20;
constexpr bool     USE_EXTERNAL_ANTENNA     = true;   // IPEX connector instead of the PCB antenna
constexpr uint32_t REJOIN_TIMEOUT_MS        = 5000;   // network already stored
constexpr uint32_t PAIRING_TIMEOUT_MS       = 60000;  // fresh device, searching for a network
constexpr uint32_t KEEP_ALIVE_MS            = 3000;
constexpr uint32_t ACK_TIMEOUT_MS           = 2000;
constexpr uint32_t RADIO_FLUSH_MS           = 200;    // let the stack finish up before power-down
constexpr float    REPORTING_DELTA          = 0.5f;
constexpr uint32_t REPORTING_MAX_INTERVAL_S = CYCLE_MS / 1000 * HEARTBEAT_CYCLES;
static_assert(REPORTING_MAX_INTERVAL_S <= UINT16_MAX, "ZCL reporting interval is 16-bit");

// Remembers across power loss whether we have ever joined, which picks the connect timeout.
constexpr const char *NVS_NAMESPACE  = "wetordead";
constexpr const char *NVS_KEY_JOINED = "joined";

// Both branches are always compiled, so debug-only code cannot silently rot.
#define LOG(...) do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)

// =============================================================================
// State retained across deep sleep
//
// RTC memory survives deep sleep but is re-initialized on every other kind of
// reset (power-on, reset button, crash), which is exactly the scope we want.
// =============================================================================

struct RetainedState {
    uint32_t bootCount;            // wakes since the last cold boot
    uint32_t cyclesSinceReport;    // wakes since the last acknowledged report
    uint32_t consecutiveFailures;  // drives back-off and the network reset
    bool     hasReported;          // lastReportedMoisture is valid
    float    lastReportedMoisture; // %, as measured
    float    lastSentMoisture = NAN;  // %, exactly as put on the air (see distinctFrom())
    float    lastSentBattery  = NAN;  // %
    uint32_t totalFailures;        // diagnostics only
    uint32_t totalNetworkResets;   // diagnostics only
};

RTC_DATA_ATTR RetainedState retained = {};

// =============================================================================
// Small helpers
// =============================================================================

void beginSerialLog() {
    if (DEBUG_SERIAL) {
        Serial.begin(115200);
        delay(3000);  // give the host time to reattach to USB-CDC
    }
}

// Waits for `done` to become true, giving up `timeoutMs` after `startMs`.
template <typename Predicate>
bool waitFor(Predicate done, uint32_t startMs, uint32_t timeoutMs) {
    while (!done()) {
        if (millis() - startMs >= timeoutMs)
            return false;
        delay(10);
    }
    return true;
}

// Drives a pin and latches it, so the level holds through light sleep.
void holdPin(uint8_t pin, uint8_t level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
    gpio_hold_en(static_cast<gpio_num_t>(pin));
}

// Undoes holdPin(): returns to the idle level, then floats the pin.
void releasePin(uint8_t pin, uint8_t idleLevel) {
    gpio_hold_dis(static_cast<gpio_num_t>(pin));
    digitalWrite(pin, idleLevel);
    pinMode(pin, INPUT);
}

void lightSleep(uint32_t ms) {
    esp_sleep_enable_timer_wakeup(uint64_t(ms) * 1000);
    esp_light_sleep_start();
    beginSerialLog();  // USB-CDC does not survive light sleep
}

float readMillivolts(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++)
        sum += analogReadMilliVolts(pin);
    return float(sum) / ADC_SAMPLES;
}

// Rounds to the reporting step, so "same value" means the same value in Home Assistant.
float quantize(float value, float step) {
    return roundf(value / step) * step;
}

// Home Assistant only records a new state when the value changes. If `value`
// equals what was last sent, move it by one step (down when at the top of the
// range), so every report - heartbeats included - shows up as a change.
float distinctFrom(float lastSent, float value, float step, float max) {
    if (std::isnan(lastSent) || fabsf(value - lastSent) >= step / 2)  // NAN: nothing sent yet
        return value;
    return value + step <= max ? value + step : value - step;
}

// =============================================================================
// Sensors
// =============================================================================

struct Reading {
    float millivolts;
    float percent;
};

float moisturePercent(float mv) {
    const float percent = (PROBE_DRY_MV - mv) / (PROBE_DRY_MV - PROBE_WET_MV) * 100.0f;
    return std::clamp(percent, 0.0f, 100.0f);
}

// Piecewise-linear interpolation over BATTERY_CURVE_MV.
float batteryPercent(float mv) {
    const auto &curve = BATTERY_CURVE_MV;
    if (mv <= curve.front())
        return 0.0f;
    if (mv >= curve.back())
        return 100.0f;

    const float stepPercent = 100.0f / (curve.size() - 1);
    size_t i = 1;
    while (mv >= curve[i])
        i++;
    const float fraction = (mv - curve[i - 1]) / (curve[i] - curve[i - 1]);
    return (i - 1 + fraction) * stepPercent;
}

Reading readSoil() {
    holdPin(PIN_PROBE_POWER, HIGH);
    if (DEBUG_MODE)
        holdPin(PIN_LED, LOW);

    lightSleep(PROBE_WARMUP_MS);

    if (DEBUG_MODE)
        releasePin(PIN_LED, HIGH);
    const float mv = readMillivolts(PIN_PROBE);
    releasePin(PIN_PROBE_POWER, LOW);

    return {mv, moisturePercent(mv)};
}

Reading readBattery() {
    const float mv = readMillivolts(PIN_BATTERY) * BATTERY_DIVIDER_RATIO;
    return {mv, batteryPercent(mv)};
}

// =============================================================================
// Persistent join flag
// =============================================================================

bool loadJoinedFlag() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
    const bool joined = prefs.getBool(NVS_KEY_JOINED, false);
    prefs.end();
    return joined;
}

void storeJoinedFlag(bool joined) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    prefs.putBool(NVS_KEY_JOINED, joined);
    prefs.end();
}

// =============================================================================
// Zigbee
// =============================================================================

ZigbeeAnalog sensorEndpoint(ZIGBEE_ENDPOINT);

// Reports still waiting for the coordinator's default response. Written from
// the Zigbee task, read from ours.
constexpr uint32_t ACK_MOISTURE = 1 << 0;
constexpr uint32_t ACK_BATTERY  = 1 << 1;
std::atomic<uint32_t> pendingAcks{0};

void onDefaultResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
    if (command != ZB_CMD_REPORT_ATTRIBUTE || endpoint != ZIGBEE_ENDPOINT || status != ESP_ZB_ZCL_STATUS_SUCCESS)
        return;
    if (cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT)
        pendingAcks.fetch_and(~ACK_MOISTURE);
    else if (cluster == ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG)
        pendingAcks.fetch_and(~ACK_BATTERY);
}

void configureRadio() {
    // The board variant already powers the RF switch and selects the PCB antenna.
    if (USE_EXTERNAL_ANTENNA)
        digitalWrite(WIFI_ANT_CONFIG, HIGH);
    esp_zb_set_tx_power(TX_POWER_DBM);
}

void configureEndpoint(const Reading &battery) {
    sensorEndpoint.setManufacturerAndModel("Espressif", "WetOrDead");
    sensorEndpoint.addAnalogInput();
    sensorEndpoint.setAnalogInputDescription("Humidity");
    sensorEndpoint.setAnalogInputApplication(ESP_ZB_ZCL_AI_HUMIDITY_SPACE);
    sensorEndpoint.setAnalogInputMinMax(0.0f, 100.0f);
    sensorEndpoint.setAnalogInputResolution(0.1f);
    // Battery voltage is not reportable; it is only published here, in 100 mV units.
    sensorEndpoint.setPowerSource(ZB_POWER_SOURCE_BATTERY, static_cast<uint8_t>(lroundf(battery.percent)),
                                  static_cast<uint8_t>(lroundf(battery.millivolts / 100.0f)));

    Zigbee.onGlobalDefaultResponse(onDefaultResponse);
    Zigbee.addEndpoint(&sensorEndpoint);
}

bool connect(bool eraseNetwork, bool wasJoined) {
    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = KEEP_ALIVE_MS;
    // Long enough that the parent keeps us through the longest back-off sleep.
    zigbeeConfig.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16384MIN;

    // begin() blocks until a stored network is rejoined, but returns as soon as
    // steering starts on a fresh device; connected() is the real signal. Both
    // waits share one deadline.
    const uint32_t timeoutMs = wasJoined ? REJOIN_TIMEOUT_MS : PAIRING_TIMEOUT_MS;
    const uint32_t startMs = millis();
    Zigbee.setTimeout(timeoutMs);

    if (!Zigbee.begin(&zigbeeConfig, eraseNetwork)) {
        LOG("Zigbee.begin() failed\r\n");
        return false;
    }
    if (!waitFor([] { return Zigbee.connected(); }, startMs, timeoutMs)) {
        LOG("Zigbee connect timeout (%" PRIu32 " ms)\r\n", timeoutMs);
        return false;
    }
    LOG("Zigbee connected in %" PRIu32 " ms\r\n", millis() - startMs);
    return true;
}

// Returns true once the coordinator has acknowledged the moisture report.
// Battery is best-effort: waited for, but a missing ack is not a failure.
bool report(float moisture, uint8_t battery) {
    // Set right after joining, so ZHA's post-join interview reads real values.
    sensorEndpoint.setAnalogInput(moisture);
    sensorEndpoint.setBatteryPercentage(battery);
    // Must run after the stack is up, or the reporting-table write panics
    // with "ZB OSIF: Zigbee lock is not ready!".
    sensorEndpoint.setAnalogInputReporting(0, REPORTING_MAX_INTERVAL_S, REPORTING_DELTA);

    // Arm before sending: an ack may arrive before report*() returns.
    pendingAcks.store(ACK_MOISTURE | ACK_BATTERY);
    if (!sensorEndpoint.reportAnalogInput())
        return false;
    if (!sensorEndpoint.reportBatteryPercentage())
        pendingAcks.fetch_and(~ACK_BATTERY);

    waitFor([] { return pendingAcks.load() == 0; }, millis(), ACK_TIMEOUT_MS);

    const uint32_t missing = pendingAcks.load();
    if (missing & ACK_BATTERY)
        LOG("Battery report not acknowledged\r\n");
    return !(missing & ACK_MOISTURE);
}

bool transmit(const Reading &soil, const Reading &battery) {
    const bool resetNetwork = retained.consecutiveFailures >= NETWORK_RESET_THRESHOLD;
    bool joined = loadJoinedFlag();
    if (resetNetwork) {
        retained.consecutiveFailures = 0;  // one reset per streak, not one per wake
        retained.totalNetworkResets++;
        LOG("!! Network reset (totalNetworkResets=%" PRIu32 ") !!\r\n", retained.totalNetworkResets);
        if (joined) {
            storeJoinedFlag(false);
            joined = false;
        }
    }

    configureRadio();
    configureEndpoint(battery);
    if (!connect(resetNetwork, joined))
        return false;
    if (!joined)
        storeJoinedFlag(true);

    const float moisture = distinctFrom(retained.lastSentMoisture, quantize(soil.percent, MOISTURE_STEP), MOISTURE_STEP, 100.0f);
    const float batteryPercent = distinctFrom(retained.lastSentBattery, quantize(battery.percent, BATTERY_STEP), BATTERY_STEP, 100.0f);
    const bool acknowledged = report(moisture, static_cast<uint8_t>(lroundf(batteryPercent)));
    // Remembered even without an ack: the coordinator may have received it anyway.
    retained.lastSentMoisture = moisture;
    retained.lastSentBattery = batteryPercent;
    delay(RADIO_FLUSH_MS);
    return acknowledged;
}

// =============================================================================
// Cycle bookkeeping
// =============================================================================

bool isReportDue(float moisture) {
    return !retained.hasReported
        || retained.cyclesSinceReport >= HEARTBEAT_CYCLES
        || fabsf(moisture - retained.lastReportedMoisture) >= MOISTURE_REPORT_THRESHOLD;
}

void recordSuccess(float moisture) {
    retained.hasReported = true;
    retained.lastReportedMoisture = moisture;
    retained.cyclesSinceReport = 0;
    retained.consecutiveFailures = 0;
}

// The report stays due, so it is retried on every (backed-off) wake.
void recordFailure() {
    retained.consecutiveFailures++;
    retained.totalFailures++;
}

// Power-on, reset button, or a fresh flash: someone may want to upload firmware.
bool isUserInitiatedBoot() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
        case ESP_RST_EXT:
        case ESP_RST_USB:
        case ESP_RST_JTAG:
            return true;
        case ESP_RST_UNKNOWN:
            return retained.bootCount == 1;
        default:
            return false;
    }
}

uint32_t cycleIntervalMs() {
    const uint32_t shift = std::min(retained.consecutiveFailures, BACKOFF_MAX_SHIFT);
    return std::min<uint64_t>(uint64_t(CYCLE_MS) << shift, BACKOFF_MAX_MS);
}

// Sleeps so that wakes stay CYCLE_MS apart (more while backing off), regardless
// of how long this wake took.
[[noreturn]] void deepSleep() {
    const uint32_t intervalMs = cycleIntervalMs();
    const uint32_t awakeMs = millis();
    const uint32_t sleepMs = intervalMs > awakeMs + MIN_SLEEP_MS ? intervalMs - awakeMs : MIN_SLEEP_MS;

    LOG("Sleeping %" PRIu32 " s (consecutiveFailures=%" PRIu32 ")\r\n", sleepMs / 1000, retained.consecutiveFailures);
    if (DEBUG_SERIAL)
        Serial.flush();

    esp_sleep_enable_timer_wakeup(uint64_t(sleepMs) * 1000);
    esp_deep_sleep_start();
}

// =============================================================================
// Entry points
// =============================================================================

void setup() {
    retained.bootCount++;
    retained.cyclesSinceReport++;

    beginSerialLog();
    LOG("\r\n=== Boot #%" PRIu32 " (consecutiveFailures=%" PRIu32 ", totalFailures=%" PRIu32 ", totalNetworkResets=%" PRIu32 ") ===\r\n",
        retained.bootCount, retained.consecutiveFailures, retained.totalFailures, retained.totalNetworkResets);

    if (isUserInitiatedBoot())
        delay(UPLOAD_WINDOW_MS);

    analogSetAttenuation(ADC_11db);  // full 0-3.1 V input range

    const Reading soil = readSoil();
    LOG("Moisture: %.2f%% (%.0f mV)\r\n", soil.percent, soil.millivolts);

    if (!isReportDue(soil.percent)) {
        LOG("No significant change\r\n");
        deepSleep();
    }

    const Reading battery = readBattery();
    LOG("Battery: %.2f%% (%.0f mV)\r\n", battery.percent, battery.millivolts);

    const bool delivered = transmit(soil, battery);
    if (delivered)
        recordSuccess(soil.percent);
    else
        recordFailure();

    LOG("Report %s\r\n", delivered ? "acknowledged" : "failed");
    deepSleep();
}

void loop() {
    deepSleep();  // unreachable: setup() always ends in deep sleep
}
