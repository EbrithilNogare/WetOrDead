#include "Zigbee.h"

// Config
#define TIME_TO_SLEEP 10 // s
#define BATTERY_AVERAGE_SAMPLES 16
#define POWER_SENSING_PIN  4
#define VOLTAGE_DIVIDER_RATIO 2.0f // 100k and 100k resistors

#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define REPORT_TIMEOUT 1000 // ms

RTC_DATA_ATTR int bootCount = 0;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 0;
bool resend = false;
float temperature = 0.0;    // °C
float humidity = 0.0;       // %
float batteryLevel = 0.0;   // %
float batteryVoltage = 0.0; // mV

/************************ Callbacks *****************************/
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  Serial.printf("Global response command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);
  if ((command == ZB_CMD_REPORT_ATTRIBUTE) && (endpoint == TEMP_SENSOR_ENDPOINT_NUMBER)) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--;   break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true;  break;
      default:                                        break;
    }
  }
}

/************************ Measuring *****************************/
void readTemperature() {
  temperature = temperatureRead();
}

void readHumidity(){
  int sensorValue = analogRead(A1);
  humidity = (sensorValue / 4096.0) * 100.0;
}

void readSensorBatteryVoltage(){
  uint32_t batteryVoltageSum = 0;
  for (int i = 0; i < BATTERY_AVERAGE_SAMPLES; i++) {
    batteryVoltageSum += analogReadMilliVolts(POWER_SENSING_PIN);
  }
  batteryVoltage = (batteryVoltageSum / static_cast<float>(BATTERY_AVERAGE_SAMPLES)) * VOLTAGE_DIVIDER_RATIO / 1000.0;
  batteryLevel = 100 - bootCount;
}

/************************ Sending *****************************/
void sendData() {
  int dataPieces = 0;
  zbTempSensor.setTemperature(temperature);
  dataPieces++;
  zbTempSensor.setHumidity(humidity);
  dataPieces++;
  
  dataToSend = dataPieces;
  zbTempSensor.report();

  // Wait until data was successfully sent
  unsigned long startTime = millis();
  int tries = 0;
  const int maxTries = 3;
  while (dataToSend != 0 && tries < maxTries) {
    if (resend) {
      resend = false;
      dataToSend = dataPieces;
      zbTempSensor.report();
    }
    if (millis() - startTime >= REPORT_TIMEOUT) {
      dataToSend = dataPieces;
      zbTempSensor.report();
      startTime = millis();
      tries++;
    }
    Serial.printf(".");
    delay(50);
  }

  zbTempSensor.reportBatteryPercentage();
  delay(50);
}

/********************* Setup **************************/
void setup() {
  bootCount++;

  Serial.begin(115200);
  delay(100);

  if(bootCount == 1) {
    Serial.println("First boot, waiting 10s for possible firmware upload...");
    delay(9000);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  // Measure
  readTemperature();
  readHumidity();
  readSensorBatteryVoltage();


  // Device info
  zbTempSensor.setManufacturerAndModel("Espressif", "WetOrDead");
  
  // Temperature
  zbTempSensor.setMinMaxValue(10, 50);
  zbTempSensor.setDefaultValue(10.0);
  zbTempSensor.setTolerance(1);
  
  // Humidity
  zbTempSensor.addHumiditySensor(0, 100, 1, 0.0);
 
  // Battery
  zbTempSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, batteryLevel, batteryVoltage);


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

  sendData();


  // Sleep
  Serial.println("Going to sleep now");
  long long timeToSleep_ms = max(TIME_TO_SLEEP * 1000LL - millis(), 1000LL);
  esp_sleep_enable_timer_wakeup(timeToSleep_ms*1000LL);
  esp_deep_sleep_start();
}

void loop() {}
