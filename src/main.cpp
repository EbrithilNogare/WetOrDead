#include "Zigbee.h"

#include "secrets.h"

// =============================================================================
// Configuration
// =============================================================================

// Sleep & Timing
static const uint32_t TIME_TO_SLEEP_MS       = 5 * 120 * 1000;
static const uint32_t REPORT_TIMEOUT_MS      = 1000;
static const uint32_t REPORT_RETRY_DELAY_MS  = 50;
static const uint8_t  MAX_REPORT_RETRIES     = 3;
static const uint32_t ZIGBEE_CONNECT_TIMEOUT_MS = 2000;

// Analog Pins
static const uint8_t POWER_SENSING_PIN       = 0;  // A0
static const uint8_t MOISTURE_SENSING_PIN    = 1;  // A1
static const uint8_t MOISTURE_POWER_PIN      = 2;  // D2

// Battery Measurement
static const uint8_t BATTERY_AVERAGE_SAMPLES = 16;
static const float   VOLTAGE_DIVIDER_RATIO   = (300000.0f + 100000.0f) / 300000.0f;

// Moisture Sensor Calibration (mV)
static const float MOISTURE_WET_VOLTAGE      = 3200.0f; // 1040.0f;
static const float MOISTURE_DRY_VOLTAGE      = 0.0f; // 2080.0f;

// Zigbee
static const uint8_t ZIGBEE_ENDPOINT         = 10;

// Battery discharge curve (voltage in mV for 0%, ..., 100%)
static const float BATTERY_DISCHARGE_CURVE[] = {
    3102, 3442, 3547, 3673, 3736, 3776, 3812, 3880, 3925, 3953, 4057
};

static constexpr size_t BATTERY_CURVE_SIZE = sizeof(BATTERY_DISCHARGE_CURVE) / sizeof(BATTERY_DISCHARGE_CURVE[0]);

// =============================================================================
// Global State
// =============================================================================

RTC_DATA_ATTR int bootCount = 0;

ZigbeeTempSensor zbTempSensor(ZIGBEE_ENDPOINT);

uint8_t dataToSend         = 0;
bool    resend             = false;

float   temperature        = 0.0f;  // °C
float   moisturePercentage = 0.0f;  // %
float   moistureVoltage    = 0.0f;  // mV
float   batteryPercentage  = 0.0f;  // %
float   batteryVoltage     = 0.0f;  // mV

float lastCycleTimings[10];

// =============================================================================
// Callbacks
// =============================================================================

void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
    log_i("Global response - command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);

    if (command != ZB_CMD_REPORT_ATTRIBUTE || endpoint != ZIGBEE_ENDPOINT) {
        return;
    }

    switch (status) {
        case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--;  break;
        case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
        default:                                       break;
    }
}

// =============================================================================
// Sensor Reading
// =============================================================================

void readTemperature() {
    temperature = temperatureRead();
}

void readHumidity() {
    pinMode(MOISTURE_POWER_PIN, OUTPUT);
    digitalWrite(MOISTURE_POWER_PIN, HIGH);

    esp_sleep_enable_timer_wakeup(100 * 1000);  // 100ms
    esp_light_sleep_start();

    moistureVoltage = analogReadMilliVolts(MOISTURE_SENSING_PIN);
    moisturePercentage = 100.0f - ((moistureVoltage - MOISTURE_WET_VOLTAGE) / (MOISTURE_DRY_VOLTAGE - MOISTURE_WET_VOLTAGE)) * 100.0f;
    moisturePercentage = constrain(moisturePercentage, 0.0f, 100.0f);

    digitalWrite(MOISTURE_POWER_PIN, LOW);
    pinMode(MOISTURE_POWER_PIN, INPUT);
}

float voltageToPercent(float voltageMv) {
    if (voltageMv <= BATTERY_DISCHARGE_CURVE[0]) {
        return 0.0f;
    }

    for (size_t i = 1; i < BATTERY_CURVE_SIZE; i++) {
        if (voltageMv < BATTERY_DISCHARGE_CURVE[i]) {
            float lowerVoltage = BATTERY_DISCHARGE_CURVE[i - 1];
            float upperVoltage = BATTERY_DISCHARGE_CURVE[i];
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
        voltageSum += analogReadMilliVolts(POWER_SENSING_PIN);
    }

    batteryVoltage = (voltageSum / static_cast<float>(BATTERY_AVERAGE_SAMPLES)) * VOLTAGE_DIVIDER_RATIO;
    batteryPercentage = voltageToPercent(batteryVoltage);
}

// =============================================================================
// Data Transmission
// =============================================================================

void sendData() {
    static constexpr uint8_t DATA_PIECES = 2;  // Temperature + Humidity

    zbTempSensor.setTemperature(temperature);
    zbTempSensor.setHumidity(moisturePercentage);

    dataToSend = DATA_PIECES;
    zbTempSensor.report();

    // Wait for successful transmission with retry logic
    unsigned long startTime = millis();
    uint8_t tries = 0;

    while (dataToSend != 0 && tries < MAX_REPORT_RETRIES) {
        if (resend) {
            resend = false;
            dataToSend = DATA_PIECES;
            zbTempSensor.report();
        }

        if (millis() - startTime >= REPORT_TIMEOUT_MS) {
            dataToSend = DATA_PIECES;
            zbTempSensor.report();
            startTime = millis();
            tries++;
        }

        delay(REPORT_RETRY_DELAY_MS);
    }

    zbTempSensor.reportBatteryPercentage();
    delay(REPORT_RETRY_DELAY_MS);
}

// =============================================================================
// Setup & Main Loop
// =============================================================================

void enterDeepSleep() {
    log_i("Going to sleep now");

    long elapsedMs = millis();
    long sleepDurationMs = TIME_TO_SLEEP_MS - elapsedMs;
    
    if(sleepDurationMs < 1000L) {
        sleepDurationMs = 1000L;
    }

    esp_sleep_enable_timer_wakeup(sleepDurationMs * 1000L);
    esp_deep_sleep_start();
}

void initializeZigbee() {
    zbTempSensor.setManufacturerAndModel("Espressif", "WetOrDead");

    // Temperature sensor configuration
    zbTempSensor.setMinMaxValue(10.0f, 50.0f);
    zbTempSensor.setDefaultValue(10.0f);
    zbTempSensor.setTolerance(1.0f);

    // Humidity sensor configuration
    zbTempSensor.addHumiditySensor(0.0f, 100.0f, 1.0f, 0.0f);

    // Power source configuration (voltage in 100mV units, e.g. 37 = 3.7V)
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, batteryPercentage, batteryVoltage / 100.0f);

    Zigbee.onGlobalDefaultResponse(onGlobalResponse);
    Zigbee.addEndpoint(&zbTempSensor);

    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
    Zigbee.setTimeout(10000);

    if (!Zigbee.begin(&zigbeeConfig, false)) {
        log_e("Zigbee failed to start!");
        log_e("Rebooting...");
        ESP.restart();
    }

    unsigned long connectStart = millis();
    while (!Zigbee.connected()) {
        if (millis() - connectStart >= ZIGBEE_CONNECT_TIMEOUT_MS) {
            log_w("Zigbee connection timed out after %lu ms, going to sleep", ZIGBEE_CONNECT_TIMEOUT_MS);
            enterDeepSleep();
        }
        log_i("Waiting for network connection");
        delay(50);
    }
}

void setup() {
    bootCount++;
    // lastCycleTimings[0] = millis();

    Serial.begin(115200);
    // lastCycleTimings[1] = millis();

    analogSetAttenuation(ADC_11db);
    // lastCycleTimings[2] = millis();

    // Allow firmware upload on first boot
    if (bootCount == 1) {
        delay(10000);
    }
    // lastCycleTimings[3] = millis();

    // Read all sensors
    readTemperature();
    // lastCycleTimings[4] = millis();

    readHumidity();
    // lastCycleTimings[5] = millis();

    readBatteryVoltage();
    // lastCycleTimings[6] = millis();

    // Initialize and connect to Zigbee network
    initializeZigbee();
    // lastCycleTimings[7] = millis();

    // Transmit sensor data
    sendData();
    // lastCycleTimings[8] = millis();

    //Serial.printf("Last cycle timings: %.2f ms\n", millis());
    //delay(2000);
    //for (size_t i = 0; i < sizeof(lastCycleTimings) / sizeof(lastCycleTimings[0]); i++) {
    //    Serial.printf("Timing %zu: %.2f ms\n", i, lastCycleTimings[i]);
    //}

    // Enter deep sleep
    enterDeepSleep();
}

void loop() {
    // Not used - device sleeps after setup
}
