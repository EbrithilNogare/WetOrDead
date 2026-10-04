// =============================================================================
// WetOrDead - battery-powered Zigbee soil-moisture sensor (Seeed XIAO ESP32-C6)
//
// Every cycle:
//
//   1. Power the probe, let it settle, sample it, power it off.
//   2. Decide whether a report is due: nothing reported yet, moisture moved by
//      MOISTURE_REPORT_THRESHOLD since the last acknowledged report, or
//      HEARTBEAT_CYCLES cycles have passed. If not, go straight back to sleep;
//      the radio is never started on those cycles.
//   3. Sample the battery, (re)join the Zigbee network, report moisture and
//      battery straight to the coordinator, and resend until it acknowledges.
//   4. Sleep until the next cycle. Consecutive failures back off
//      exponentially; too many in a row wipe the stored network so the device
//      pairs from scratch.
//
// Production builds deep-sleep between cycles, so each cycle is a fresh boot.
// DEBUG_SERIAL builds never sleep (USB-CDC would drop); loop() runs the cycles.
// =============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <Zigbee.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <zcl/esp_zigbee_zcl_power_config.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <cmath>

// =============================================================================
// Configuration
// =============================================================================

// Serial logging over USB-CDC; the device then stays awake. Enabled by the `main-debug` env.
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL false
#endif

// Bench testing: shorter cycles, and the user LED lights while the probe warms up.
#ifndef DEBUG_MODE
#define DEBUG_MODE false
#endif

// --- Cycle ---
constexpr uint32_t CYCLE_MS         = (DEBUG_MODE ? 10 : 30) * 60 * 1000;
// Report at least this often, even if unchanged. ZHA marks a battery device
// unavailable after 6 h of silence, so this leaves room for one missed heartbeat.
constexpr uint32_t HEARTBEAT_CYCLES = DEBUG_MODE ? 3 : 6;
constexpr uint32_t UPLOAD_WINDOW_MS = 10000;  // stay awake after a manual reset so USB can flash us
constexpr uint32_t MIN_SLEEP_MS     = 1000;
static_assert(uint64_t(CYCLE_MS) * HEARTBEAT_CYCLES < 6ULL * 60 * 60 * 1000, "heartbeat must beat ZHA's 6 h timeout");

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
constexpr uint16_t COORDINATOR_ADDRESS      = 0x0000;
constexpr uint8_t  COORDINATOR_ENDPOINT     = 1;      // ZHA and Zigbee2MQTT both listen here
constexpr int8_t   TX_POWER_DBM             = 20;
constexpr bool     USE_EXTERNAL_ANTENNA     = true;   // IPEX connector instead of the PCB antenna
constexpr uint32_t REJOIN_TIMEOUT_MS        = 5000;   // network already stored
constexpr uint32_t PAIRING_TIMEOUT_MS       = 60000;  // fresh device, searching for a network
constexpr uint32_t INTERVIEW_WINDOW_MS      = 60000;  // stay reachable while ZHA interviews/configures us
constexpr uint32_t KEEP_ALIVE_MS            = 10000;  // as the library's sleepy example: stays out of the way of reports
constexpr uint8_t  REPORT_ATTEMPTS          = 3;
constexpr uint32_t ACK_TIMEOUT_MS           = 1500;   // per attempt
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
// State retained between cycles
//
// RTC memory survives deep sleep but is re-initialized on every other kind of
// reset (power-on, reset button, crash), which is exactly the scope we want.
// =============================================================================

struct RetainedState {
    uint32_t bootCount;            // cycles since the last cold boot
    uint32_t cyclesSinceReport;    // cycles since the last acknowledged report
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
        delay(3000);  // give the host time to attach to USB-CDC
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

// Idles at low power. DEBUG_SERIAL builds just wait, since USB-CDC drops in light sleep.
void lightSleep(uint32_t ms) {
    if (DEBUG_SERIAL) {
        delay(ms);
        return;
    }
    esp_sleep_enable_timer_wakeup(uint64_t(ms) * 1000);
    esp_light_sleep_start();
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

// Set once Zigbee.begin() has been called; it must not be called twice per boot.
// (Tracked here because ZigbeeCore::initialized() only exists in newer cores.)
bool zigbeeBegun = false;

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
    sensorEndpoint.setAnalogInputResolution(MOISTURE_STEP);
    sensorEndpoint.setPowerSource(ZB_POWER_SOURCE_BATTERY, static_cast<uint8_t>(lroundf(battery.percent)),
                                  static_cast<uint8_t>(lroundf(battery.millivolts / 100.0f)));

    Zigbee.onGlobalDefaultResponse(onDefaultResponse);
    Zigbee.addEndpoint(&sensorEndpoint);
}

// Starts the stack on first use (once per boot) and waits for the network.
bool connect(const Reading &battery, bool eraseNetwork, bool wasJoined) {
    const uint32_t timeoutMs = wasJoined ? REJOIN_TIMEOUT_MS : PAIRING_TIMEOUT_MS;
    const uint32_t startMs = millis();

    if (!zigbeeBegun) {
        zigbeeBegun = true;
        configureRadio();
        configureEndpoint(battery);

        esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
        zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = KEEP_ALIVE_MS;
        // Long enough that the parent keeps us through the longest back-off sleep.
        zigbeeConfig.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16384MIN;
        // Receiver on while awake, so the coordinator's replies arrive at once
        // instead of waiting for the next poll. Asleep we are unreachable anyway.
        Zigbee.setRxOnWhenIdle(true);
        Zigbee.setTimeout(timeoutMs);

        // begin() blocks until a stored network is rejoined, but returns as soon
        // as steering starts on a fresh device; connected() is the real signal.
        if (!Zigbee.begin(&zigbeeConfig, eraseNetwork)) {
            LOG("Zigbee.begin() failed\r\n");
            return false;
        }
    }
    if (!waitFor([] { return Zigbee.connected(); }, startMs, timeoutMs)) {
        LOG("Zigbee connect timeout (%" PRIu32 " ms)\r\n", timeoutMs);
        return false;
    }
    LOG("Zigbee connected in %" PRIu32 " ms\r\n", millis() - startMs);
    return true;
}

// Sends one attribute report straight to the coordinator. The library's own
// report*() functions go through the binding table instead, which only works
// if the coordinator managed to bind us while we were awake after pairing.
bool sendReport(uint16_t cluster, uint16_t attribute) {
    esp_zb_zcl_report_attr_cmd_t cmd = {};
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = COORDINATOR_ADDRESS;
    cmd.zcl_basic_cmd.dst_endpoint = COORDINATOR_ENDPOINT;
    cmd.zcl_basic_cmd.src_endpoint = ZIGBEE_ENDPOINT;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = cluster;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    cmd.dis_default_resp = 0;  // the default response is our delivery receipt
    cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    cmd.attributeID = attribute;

    esp_zb_lock_acquire(portMAX_DELAY);
    const esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&cmd);
    esp_zb_lock_release();
    return err == ESP_OK;
}

void sendPendingReports() {
    const uint32_t pending = pendingAcks.load();
    if ((pending & ACK_MOISTURE) && !sendReport(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID))
        LOG("Moisture report not queued\r\n");
    if ((pending & ACK_BATTERY) && !sendReport(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID))
        LOG("Battery report not queued\r\n");
}

// Returns true once the coordinator has acknowledged the moisture report.
// Battery is best-effort: resent and waited for, but a missing ack is not a failure.
bool report(float moisture, uint8_t battery, const Reading &batteryReading) {
    sensorEndpoint.setAnalogInput(moisture);
    sensorEndpoint.setBatteryPercentage(battery);
    sensorEndpoint.setBatteryVoltage(static_cast<uint8_t>(lroundf(batteryReading.millivolts / 100.0f)));
    // Must run after the stack is up, or the reporting-table write panics
    // with "ZB OSIF: Zigbee lock is not ready!".
    sensorEndpoint.setAnalogInputReporting(0, REPORTING_MAX_INTERVAL_S, REPORTING_DELTA);

    // Arm before sending: an ack may arrive before sendReport() returns.
    pendingAcks.store(ACK_MOISTURE | ACK_BATTERY);
    for (uint8_t attempt = 1; attempt <= REPORT_ATTEMPTS && pendingAcks.load() != 0; attempt++) {
        if (attempt > 1)
            LOG("Resending unacknowledged reports (attempt %u)\r\n", attempt);
        sendPendingReports();
        waitFor([] { return pendingAcks.load() == 0; }, millis(), ACK_TIMEOUT_MS);
    }

    const uint32_t missing = pendingAcks.load();
    if (missing & ACK_BATTERY)
        LOG("Battery report not acknowledged\r\n");
    return !(missing & ACK_MOISTURE);
}

bool transmit(const Reading &soil, const Reading &battery, bool firstSinceBoot) {
    const bool resetNetwork = retained.consecutiveFailures >= NETWORK_RESET_THRESHOLD;
    bool joined = loadJoinedFlag();
    if (resetNetwork) {
        retained.consecutiveFailures = 0;  // one reset per streak, not one per cycle
        retained.totalNetworkResets++;
        LOG("!! Network reset (totalNetworkResets=%" PRIu32 ") !!\r\n", retained.totalNetworkResets);
        if (joined) {
            storeJoinedFlag(false);
            joined = false;
        }
        if (zigbeeBegun)
            Zigbee.factoryReset();  // only in DEBUG_SERIAL builds; erases and restarts
    }

    if (!connect(battery, resetNetwork, joined))
        return false;
    if (!joined)
        storeJoinedFlag(true);

    const float moisture = distinctFrom(retained.lastSentMoisture, quantize(soil.percent, MOISTURE_STEP), MOISTURE_STEP, 100.0f);
    const float batteryPercent = distinctFrom(retained.lastSentBattery, quantize(battery.percent, BATTERY_STEP), BATTERY_STEP, 100.0f);

    // After pairing, or a manual reset (e.g. to use ZHA's "Reconfigure"), stay
    // reachable so the coordinator can interview and configure us; it ignores
    // reports from endpoints it has not interviewed yet. Values are set first so
    // its attribute reads see real data.
    if (!joined || (firstSinceBoot && isUserInitiatedBoot())) {
        sensorEndpoint.setAnalogInput(moisture);
        sensorEndpoint.setBatteryPercentage(static_cast<uint8_t>(lroundf(batteryPercent)));
        LOG("Staying awake %" PRIu32 " s for the coordinator\r\n", INTERVIEW_WINDOW_MS / 1000);
        delay(INTERVIEW_WINDOW_MS);
    }

    const bool acknowledged = report(moisture, static_cast<uint8_t>(lroundf(batteryPercent)), battery);
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

// The report stays due, so it is retried on every (backed-off) cycle.
void recordFailure() {
    retained.consecutiveFailures++;
    retained.totalFailures++;
}

uint32_t cycleIntervalMs() {
    const uint32_t shift = std::min(retained.consecutiveFailures, BACKOFF_MAX_SHIFT);
    return std::min<uint64_t>(uint64_t(CYCLE_MS) << shift, BACKOFF_MAX_MS);
}

// One measure/report cycle. `firstSinceBoot` is always true in production,
// where every cycle starts from deep sleep.
void runCycle(bool firstSinceBoot) {
    retained.bootCount++;
    retained.cyclesSinceReport++;
    LOG("\r\n=== Cycle #%" PRIu32 " (consecutiveFailures=%" PRIu32 ", totalFailures=%" PRIu32 ", totalNetworkResets=%" PRIu32 ") ===\r\n",
        retained.bootCount, retained.consecutiveFailures, retained.totalFailures, retained.totalNetworkResets);

    if (firstSinceBoot && isUserInitiatedBoot())
        delay(UPLOAD_WINDOW_MS);

    const Reading soil = readSoil();
    LOG("Moisture: %.2f%% (%.0f mV)\r\n", soil.percent, soil.millivolts);

    if (!isReportDue(soil.percent)) {
        LOG("No significant change\r\n");
        return;
    }

    const Reading battery = readBattery();
    LOG("Battery: %.2f%% (%.0f mV)\r\n", battery.percent, battery.millivolts);

    const bool delivered = transmit(soil, battery, firstSinceBoot);
    if (delivered)
        recordSuccess(soil.percent);
    else
        recordFailure();
    LOG("Report %s\r\n", delivered ? "acknowledged" : "failed");
}

// Waits out the rest of the cycle so cycles start CYCLE_MS apart (more while
// backing off). Production deep-sleeps and never returns; DEBUG_SERIAL builds
// wait awake and return, and loop() runs the next cycle.
void sleepRestOfCycle(uint32_t cycleStartMs) {
    const uint32_t intervalMs = cycleIntervalMs();
    const uint32_t awakeMs = millis() - cycleStartMs;
    const uint32_t sleepMs = intervalMs > awakeMs + MIN_SLEEP_MS ? intervalMs - awakeMs : MIN_SLEEP_MS;
    LOG("Sleeping %" PRIu32 " s (consecutiveFailures=%" PRIu32 ")\r\n", sleepMs / 1000, retained.consecutiveFailures);

    if (DEBUG_SERIAL) {
        delay(sleepMs);
        return;
    }
    esp_sleep_enable_timer_wakeup(uint64_t(sleepMs) * 1000);
    esp_deep_sleep_start();
}

// =============================================================================
// Entry points
// =============================================================================

void setup() {
    beginSerialLog();
    analogSetAttenuation(ADC_11db);  // full 0-3.1 V input range

    const uint32_t startMs = millis();
    runCycle(/*firstSinceBoot=*/true);
    sleepRestOfCycle(startMs);
}

// Only reached in DEBUG_SERIAL builds; production deep-sleeps at the end of setup().
void loop() {
    const uint32_t startMs = millis();
    runCycle(/*firstSinceBoot=*/false);
    sleepRestOfCycle(startMs);
}
