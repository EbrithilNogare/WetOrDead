#include "Zigbee.h"
#include "ep/ZigbeeAnalog.h"
#include "Arduino.h"

// =============================================================================
// Configuration
// =============================================================================

#define DEBUG_MODE true

#if DEBUG_MODE
static const int TIME_TO_SLEEP_MS          = 1 * 60 * 60 * 1000; // ms
static const int FULL_UPDATE_INTERVAL      = 12;
#else
static const int TIME_TO_SLEEP_MS          = 1 * 60 * 1000; // ms
static const int FULL_UPDATE_INTERVAL      = 5;
#endif
static const int SENSOR_WARMUP_MS          = 60;            // ms
static const int REPORT_WAIT_TIMEOUT_MS    = 2000;          // ms
static const int ZIGBEE_CONNECT_TIMEOUT_MS = 15000;         // ms
static const int ZIGBEE_PAIRING_TIMEOUT_MS = 60000;         // ms
static const float MOISTURE_CHANGE_THRESHOLD = 5.0f; // % change to trigger send

// Pins
static const int PIN_BATTERY          =  0; // A0
static const int PIN_MOISTURE         =  1; // A1
static const int PIN_MOISTURE_PWR     =  2; // D2
static const int PIN_USER_LED         = 15; // XIAO ESP32C6 user LED, active-low

// Battery
static const int   BATTERY_AVERAGE_SAMPLES = 16;
static const float VOLTAGE_DIVIDER_RATIO   = (300000.0f + 100000.0f) / 300000.0f;
static const float BATTERY_CURVE[] = { 3200, 3442, 3547, 3673, 3736, 3776, 3812, 3880, 3925, 3953, 4100 };

// Moisture Sensor Calibration (mV)
static const float MOISTURE_WET_VOLTAGE = 3200.0f;
static const float MOISTURE_DRY_VOLTAGE = 0.0f;

// Zigbee
static const int ZIGBEE_ENDPOINT                = 10;
static const int ZIGBEE_FACTORY_RESET_THRESHOLD = 5;


static constexpr int BATTERY_CURVE_SIZE = sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);

// =============================================================================
// Global State
// =============================================================================

RTC_DATA_ATTR long  bootCount             = 0;
RTC_DATA_ATTR int   cyclesSinceUpdate     = 0;
RTC_DATA_ATTR int   failCount             = 0;
RTC_DATA_ATTR float lastMoisture          = -100.0f;
RTC_DATA_ATTR bool  heartbeat             = false;
RTC_DATA_ATTR long  totalFailures         = 0; // diagnostic: cumulative failed cycles
RTC_DATA_ATTR long  totalResets           = 0; // diagnostic: cumulative factory resets

ZigbeeAnalog zbAnalog(ZIGBEE_ENDPOINT);

float moisturePercentage = 0.0f; // %
float moistureVoltage    = 0.0f; // mV
float batteryPercentage  = 0.0f; // %
float batteryVoltage     = 0.0f; // mV
bool dataSendSuccessfuly = false;
bool needFactoryReset    = false;
int dataToSend           = 0;

// =============================================================================
// Sensor Reading
// =============================================================================

void moistureSensorPowerOn() {
    pinMode(PIN_MOISTURE_PWR, OUTPUT);
    digitalWrite(PIN_MOISTURE_PWR, HIGH);
    gpio_hold_en((gpio_num_t)PIN_MOISTURE_PWR);
}

void lightSleepForSensor() {
#if DEBUG_MODE
    pinMode(PIN_USER_LED, OUTPUT);
    digitalWrite(PIN_USER_LED, LOW); // active-low: LED on
    gpio_hold_en((gpio_num_t)PIN_USER_LED);
#endif

    esp_sleep_enable_timer_wakeup(SENSOR_WARMUP_MS * 1000L);
    esp_light_sleep_start();
    Serial.begin(115200); // reinit after light sleep (APB clock is gated during sleep)

#if DEBUG_MODE
    gpio_hold_dis((gpio_num_t)PIN_USER_LED);
    digitalWrite(PIN_USER_LED, HIGH); // LED off
    pinMode(PIN_USER_LED, INPUT);
#endif
}

void readMoistureAdc() {
    moistureVoltage = analogReadMilliVolts(PIN_MOISTURE);
    moisturePercentage = 100.0f - ((moistureVoltage - MOISTURE_WET_VOLTAGE) / (MOISTURE_DRY_VOLTAGE - MOISTURE_WET_VOLTAGE)) * 100.0f;
    moisturePercentage = constrain(moisturePercentage, 0.0f, 100.0f);

    gpio_hold_dis((gpio_num_t)PIN_MOISTURE_PWR);
    digitalWrite(PIN_MOISTURE_PWR, LOW);
    pinMode(PIN_MOISTURE_PWR, INPUT);
}

bool shouldSendData() {
    if (cyclesSinceUpdate >= FULL_UPDATE_INTERVAL)
        return true;

    return fabs(moisturePercentage - lastMoisture) >= MOISTURE_CHANGE_THRESHOLD;
}

float voltageToPercent(float voltageMv) {
    if (voltageMv <= BATTERY_CURVE[0])
        return 0.0f;

    for (size_t i = 1; i < BATTERY_CURVE_SIZE; i++) {
        if (voltageMv < BATTERY_CURVE[i]) {
            float lowerVoltage = BATTERY_CURVE[i - 1];
            float upperVoltage = BATTERY_CURVE[i];
            float lowerPercent = (i - 1) * 10.0f;
            float upperPercent = i * 10.0f;
            return map(voltageMv, lowerVoltage, upperVoltage, lowerPercent, upperPercent);
        }
    }

    return 100.0f;
}

void readBatteryVoltage() {
    uint32_t voltageSum = 0;

    for (uint8_t i = 0; i < BATTERY_AVERAGE_SAMPLES; i++) {
        voltageSum += analogReadMilliVolts(PIN_BATTERY);
    }

    batteryVoltage = (voltageSum / static_cast<float>(BATTERY_AVERAGE_SAMPLES)) * VOLTAGE_DIVIDER_RATIO;
    batteryPercentage = voltageToPercent(batteryVoltage);
}

// =============================================================================
// Data Transmission
// =============================================================================

void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
    if (command == ZB_CMD_REPORT_ATTRIBUTE && endpoint == ZIGBEE_ENDPOINT && status == ESP_ZB_ZCL_STATUS_SUCCESS)
        dataToSend--;
}

bool responseChecker() {
    unsigned long startMs = millis();
    while (dataToSend > 0) {
        if (millis() - startMs >= REPORT_WAIT_TIMEOUT_MS)
            return false;
        delay(10);
    }
    return true;
}

void sendData() {
    zbAnalog.setAnalogInput(moisturePercentage);
    zbAnalog.setBatteryPercentage((uint8_t)batteryPercentage);

    dataToSend = 0;
    int attemptedReports = 0;
    if (zbAnalog.reportBatteryPercentage()) { dataToSend++; attemptedReports++; } // no confirmation for battery percentage
    if (zbAnalog.reportAnalogInput())       { dataToSend++; attemptedReports++; }

    dataSendSuccessfuly = (attemptedReports > 0) && responseChecker();

    if (dataSendSuccessfuly) {
        lastMoisture = moisturePercentage;
        failCount = 0;
    } else {
        failCount++;
        totalFailures++;
    }
}

// =============================================================================
// Setup & Main Loop
// =============================================================================

void enterDeepSleep() {
    long elapsedMs = millis();
    long sleepDurationMs = TIME_TO_SLEEP_MS - elapsedMs;

    if(sleepDurationMs < 1000L) {
        sleepDurationMs = 1000L;
    }

    esp_sleep_enable_timer_wakeup(sleepDurationMs * 1000L);
    esp_deep_sleep_start();
}

void checkStability() {
    if (failCount >= ZIGBEE_FACTORY_RESET_THRESHOLD) {
        needFactoryReset = true;
        totalResets++;
        failCount = 0; // reset counter so we don't loop-reset every wake
        Serial.printf("!! Factory reset triggered (totalResets=%ld) !!\r\n", totalResets);
        Zigbee.factoryReset(false);

        pinMode(PIN_USER_LED, OUTPUT);
        for (int i = 0; i < 10; i++) {
            digitalWrite(PIN_USER_LED, LOW);
            delay(200);
            digitalWrite(PIN_USER_LED, HIGH);
            delay(200);
        }
        pinMode(PIN_USER_LED, INPUT);
    }
}

void initializeZigbee() {
    zbAnalog.setManufacturerAndModel("Espressif", "WetOrDead");
    zbAnalog.addAnalogInput();
    zbAnalog.setAnalogInputDescription("Humidity");
    zbAnalog.setAnalogInputApplication(ESP_ZB_ZCL_AI_HUMIDITY_SPACE);
    zbAnalog.setAnalogInputMinMax(0.0f, 100.0f);
    zbAnalog.setAnalogInputResolution(0.1f);
    zbAnalog.setAnalogInputReporting(0, TIME_TO_SLEEP_MS / 1000 * FULL_UPDATE_INTERVAL, 0.5f);

    zbAnalog.setPowerSource(ZB_POWER_SOURCE_BATTERY, (uint8_t)batteryPercentage, (uint8_t)(batteryVoltage / 100.0f));

    Zigbee.onGlobalDefaultResponse(onGlobalResponse);
    Zigbee.addEndpoint(&zbAnalog);

    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    uint32_t connectTimeout = (bootCount == 1 || needFactoryReset) ? ZIGBEE_PAIRING_TIMEOUT_MS : ZIGBEE_CONNECT_TIMEOUT_MS;
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 3000;
    zigbeeConfig.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16384MIN;
    Zigbee.setTimeout(connectTimeout);

    if (!Zigbee.begin(&zigbeeConfig, needFactoryReset)) {
        Serial.println("Zigbee.begin() failed");
        failCount++;
        totalFailures++;
        enterDeepSleep();
        return;
    }

    unsigned long connectStart = millis();
    while (!Zigbee.connected()) {
        if (millis() - connectStart >= connectTimeout) {
            Serial.printf("Zigbee connect timeout (%lums)\r\n", (unsigned long)connectTimeout);
            failCount++;
            totalFailures++;
            enterDeepSleep();
            return;
        }
        delay(20);
    }
    Serial.printf("Zigbee connected in %lums\r\n", millis() - connectStart);
}

void setupAntenna() {
    esp_zb_set_tx_power(20); // dBm
    // Switch RF to external antenna (XIAO ESP32C6 FM8625H RF switch)
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);   // Power on the RF switch
    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH); // Select external antenna
}

void forceHeartbeat()
{
    heartbeat = !heartbeat;
    if (heartbeat) moisturePercentage += 0.1f;
    if (heartbeat) batteryPercentage += 1.0f;
}

void setup() {
    bootCount++;
    cyclesSinceUpdate++;

    Serial.begin(115200);
    Serial.printf("\r\n=== Boot #%ld (failCount=%d, totalFailures=%ld, totalResets=%ld) ===\r\n",
                  bootCount, failCount, totalFailures, totalResets);

    // Allow firmware upload on first boot
    if (bootCount == 1) {
        delay(10000);
    }

    analogSetAttenuation(ADC_11db);
    setupAntenna();

    moistureSensorPowerOn();
    lightSleepForSensor();
    readMoistureAdc();

    Serial.printf("Moisture: %.2f%% (%.0f mV)\r\n", moisturePercentage, moistureVoltage);

    if (!shouldSendData()) {
        // Small change -> go back to sleep
        Serial.println("No significant change, sleeping...");
        enterDeepSleep();
        return;
    }
    cyclesSinceUpdate = 0;

    readBatteryVoltage();
    Serial.printf("Battery: %.2f%% (%.0f mV)\r\n", batteryPercentage, batteryVoltage);

    checkStability();
    Serial.println("Connecting to Zigbee...");
    initializeZigbee();

    forceHeartbeat();
    sendData();

    Serial.printf("Data send %s. Moisture: %.2f%%, Battery: %.2f%%, failCount=%d\r\n",
                  dataSendSuccessfuly ? "successful" : "failed",
                  moisturePercentage, batteryPercentage, failCount);
    delay(200);

    enterDeepSleep();
}

void loop() {
    // Not used - device sleeps after setup
}
