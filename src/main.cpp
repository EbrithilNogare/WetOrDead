#include "Zigbee.h"

#include "secrets.h"

// =============================================================================
// Configuration
// =============================================================================

// Sleep & Timing
static const int TIME_TO_SLEEP_MS          = 1 * 10 * 1000; // ms
static const int SENSOR_WARMUP_MS          = 60;           // ms
static const int REPORT_SEND_DELAY_MS      = 100;           // ms
static const int ZIGBEE_CONNECT_TIMEOUT_MS = 2000;          // ms
static const int ZIGBEE_PAIRING_TIMEOUT_MS = 30000;         // ms

// Change detection
static const float MOISTURE_CHANGE_THRESHOLD = 5.0f; // % change to trigger send
static const int   FULL_UPDATE_INTERVAL      = 2;    // force send every N small cycles

// Analog Pins
static const int POWER_SENSING_PIN    = 0; // A0
static const int MOISTURE_SENSING_PIN = 1; // A1
static const int MOISTURE_POWER_PIN   = 2; // D2

// Battery Measurement
static const int   BATTERY_AVERAGE_SAMPLES = 16;
static const float VOLTAGE_DIVIDER_RATIO   = (300000.0f + 100000.0f) / 300000.0f;

// Moisture Sensor Calibration (mV)
static const float MOISTURE_WET_VOLTAGE = 3200.0f; // 1040.0f;
static const float MOISTURE_DRY_VOLTAGE = 0.0f;    // 2080.0f;

// Zigbee
static const int ZIGBEE_ENDPOINT = 10;

// Battery discharge curve (voltage in mV for 0%, ..., 100%)
static const float BATTERY_DISCHARGE_CURVE[] = {
    3100, 3442, 3547, 3673, 3736, 3776, 3812, 3880, 3925, 3953, 4100
};

static constexpr int BATTERY_CURVE_SIZE = sizeof(BATTERY_DISCHARGE_CURVE) / sizeof(BATTERY_DISCHARGE_CURVE[0]);

// =============================================================================
// Global State
// =============================================================================

RTC_DATA_ATTR long  bootCount             = 0;
RTC_DATA_ATTR float previousMoistureValue = -100.0f;
RTC_DATA_ATTR int   smallCycleCount       = 0;        // cycles since last full update

ZigbeeTempSensor zbTempSensor(ZIGBEE_ENDPOINT);

float temperature        = 0.0f; // °C
float moisturePercentage = 0.0f; // %
float moistureVoltage    = 0.0f; // mV
float batteryPercentage  = 0.0f; // %
float batteryVoltage     = 0.0f; // mV

// =============================================================================
// Sensor Reading
// =============================================================================

void readTemperature() {
    temperature = temperatureRead();
}

void moistureSensorPowerOn() {
    pinMode(MOISTURE_POWER_PIN, OUTPUT);
    digitalWrite(MOISTURE_POWER_PIN, HIGH);
    gpio_hold_en((gpio_num_t)MOISTURE_POWER_PIN);
}

void lightSleepForSensor() {
    esp_sleep_enable_timer_wakeup(SENSOR_WARMUP_MS * 1000L);
    esp_light_sleep_start();
}

void readMoistureAdc() {
    moistureVoltage = analogReadMilliVolts(MOISTURE_SENSING_PIN);
    moisturePercentage = 100.0f - ((moistureVoltage - MOISTURE_WET_VOLTAGE) / (MOISTURE_DRY_VOLTAGE - MOISTURE_WET_VOLTAGE)) * 100.0f;
    moisturePercentage = constrain(moisturePercentage, 0.0f, 100.0f);

    gpio_hold_dis((gpio_num_t)MOISTURE_POWER_PIN);
    digitalWrite(MOISTURE_POWER_PIN, LOW);
    pinMode(MOISTURE_POWER_PIN, INPUT);
}

bool shouldSendData() {
    if (smallCycleCount >= FULL_UPDATE_INTERVAL)
        return true;

    return fabs(moisturePercentage - previousMoistureValue) >= MOISTURE_CHANGE_THRESHOLD;
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
    zbTempSensor.setTemperature(temperature);
    zbTempSensor.setHumidity(moisturePercentage);
    zbTempSensor.setBatteryPercentage(batteryPercentage);

    bool success = zbTempSensor.report();
    success &= zbTempSensor.reportBatteryPercentage();

    delay(1);
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
    zbTempSensor.setMinMaxValue(-10.0f, 60.0f);
    zbTempSensor.setDefaultValue(10.0f);
    zbTempSensor.setTolerance(1.0f);

    // Humidity sensor configuration
    zbTempSensor.addHumiditySensor(0.0f, 100.0f, 1.0f, 0.0f);

    batteryPercentage = bootCount % 100; // TODO remove this debug code

    // Power source configuration (voltage in 100mV units, e.g. 37 = 3.7V)
    zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, batteryPercentage, batteryVoltage / 100.0f);

    Zigbee.addEndpoint(&zbTempSensor);

    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    uint32_t connectTimeout = bootCount == 1 ? ZIGBEE_PAIRING_TIMEOUT_MS : ZIGBEE_CONNECT_TIMEOUT_MS;
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = connectTimeout;
    Zigbee.setTimeout(connectTimeout);

    if (!Zigbee.begin(&zigbeeConfig, false)) {
        log_e("Zigbee failed to start, going to sleep");
        smallCycleCount++;
        enterDeepSleep();
    }

    unsigned long connectStart = millis();
    while (!Zigbee.connected()) {
        if (millis() - connectStart >= connectTimeout) {
            log_w("Zigbee connection timed out after %lu ms, going to sleep", connectTimeout);
            enterDeepSleep();
        }
        log_i("Waiting for network connection");
        delay(20);
    }
}

void setupAntenna() {
    esp_zb_set_tx_power(20); // dBm

    // Switch RF to external antenna (XIAO ESP32C6 FM8625H RF switch)
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);   // Power on the RF switch
    pinMode(14, OUTPUT);
    digitalWrite(14, HIGH); // Select external antenna
}

void setup() {
    bootCount++;
    smallCycleCount++;

    Serial.begin(115200);

    // Allow firmware upload on first boot
    if (bootCount == 1) {
        delay(10000);
    }
    
    analogSetAttenuation(ADC_11db);
    setupAntenna();

    moistureSensorPowerOn();
    lightSleepForSensor();
    readMoistureAdc();

    if (!shouldSendData()) {
        // Small change -> go back to sleep
        enterDeepSleep();
        return;
    }
    smallCycleCount = 0;

    readTemperature();
    readBatteryVoltage();
    initializeZigbee();
    sendData();
    previousMoistureValue = moisturePercentage;

    enterDeepSleep();
}

void loop() {
    // Not used - device sleeps after setup
}
