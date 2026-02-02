#include "Zigbee.h"

// =============================================================================
// Configuration
// =============================================================================

// Sleep & Timing
static constexpr uint32_t TIME_TO_SLEEP_MS       = 10 * 1000;
static constexpr uint32_t REPORT_TIMEOUT_MS      = 1000;
static constexpr uint32_t REPORT_RETRY_DELAY_MS  = 50;
static constexpr uint8_t  MAX_REPORT_RETRIES     = 3;

// Analog Pins
static constexpr uint8_t POWER_SENSING_PIN    = 0;  // A0
static constexpr uint8_t MOISTURE_SENSING_PIN = 1;  // A1

// Battery Measurement
static constexpr uint8_t BATTERY_AVERAGE_SAMPLES = 16;
static constexpr float   VOLTAGE_DIVIDER_RATIO   = (300000.0f + 100000.0f) / 300000.0f;

// Zigbee
static constexpr uint8_t ZIGBEE_ENDPOINT = 10;

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

static uint8_t dataToSend     = 0;
static bool    resend         = false;
static float   temperature    = 0.0f;  // °C
static float   humidity       = 0.0f;  // %
static float   batteryLevel   = 0.0f;  // %
static float   batteryVoltage = 0.0f;  // mV

// =============================================================================
// Callbacks
// =============================================================================

void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
    Serial.printf("Global response - command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);

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
    int sensorValue = analogRead(MOISTURE_SENSING_PIN);
    humidity = (sensorValue / 4096.0f) * 100.0f;
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
    batteryLevel = voltageToPercent(batteryVoltage);
}

// =============================================================================
// Data Transmission
// =============================================================================

void sendData() {
    static constexpr uint8_t DATA_PIECES = 2;  // Temperature + Humidity

    zbTempSensor.setTemperature(temperature);
    zbTempSensor.setHumidity(humidity);

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

        Serial.print(".");
        delay(REPORT_RETRY_DELAY_MS);
    }

    zbTempSensor.reportBatteryPercentage();
    delay(REPORT_RETRY_DELAY_MS);
}

// =============================================================================
// Setup & Main Loop
// =============================================================================

void initializeZigbee() {
    zbTempSensor.setManufacturerAndModel("Espressif", "WetOrDead");

    // Temperature sensor configuration
    zbTempSensor.setMinMaxValue(10, 50);
    zbTempSensor.setDefaultValue(10.0f);
    zbTempSensor.setTolerance(1);

    // Humidity sensor configuration
    zbTempSensor.addHumiditySensor(0, 100, 1, 0.0f);

    // Power source configuration
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, batteryLevel, batteryVoltage / 1000.0f);

    Zigbee.onGlobalDefaultResponse(onGlobalResponse);
    Zigbee.addEndpoint(&zbTempSensor);

    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;
    Zigbee.setTimeout(10000);

    if (!Zigbee.begin(&zigbeeConfig, false)) {
        Serial.println("Zigbee failed to start!");
        Serial.println("Rebooting...");
        ESP.restart();
    }

    Serial.println("Connecting to network");
    while (!Zigbee.connected()) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    Serial.println("Successfully connected to Zigbee network");
}

void enterDeepSleep() {
    Serial.println("Going to sleep now");

    int64_t elapsedMs = static_cast<int64_t>(millis());
    int64_t sleepDurationMs = max(TIME_TO_SLEEP_MS - elapsedMs, 1000LL);

    esp_sleep_enable_timer_wakeup(sleepDurationMs * 1000LL);
    esp_deep_sleep_start();
}

void setup() {
    bootCount++;
    Serial.begin(115200);
    delay(100);

    // Allow firmware upload on first boot
    if (bootCount == 1) {
        Serial.println("First boot, waiting 10s for possible firmware upload...");
        delay(9000);
    }

    // Visual indicator
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    // Read all sensors
    readTemperature();
    readHumidity();
    readBatteryVoltage();

    // Initialize and connect to Zigbee network
    initializeZigbee();

    // Transmit sensor data
    sendData();

    // Enter deep sleep
    enterDeepSleep();
}

void loop() {
    // Not used - device sleeps after setup
}
