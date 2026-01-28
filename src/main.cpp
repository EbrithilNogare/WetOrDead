#include "Zigbee.h"
#include "driver/temperature_sensor.h"

#define ZIGBEE_TEMP_ENDPOINT 10
uint8_t button = BOOT_PIN;

temperature_sensor_handle_t temp_sensor = NULL;
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(ZIGBEE_TEMP_ENDPOINT);

void setup() {
  Serial.begin(115200);

  pinMode(button, INPUT_PULLUP);

  temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
  if (temperature_sensor_install(&temp_sensor_config, &temp_sensor) == ESP_OK) {
    temperature_sensor_enable(temp_sensor);
  }

  zbTempSensor.setManufacturerAndModel("Espressif", "ZBTempSensor");

  Zigbee.addEndpoint(&zbTempSensor);

  if (!Zigbee.begin()) {
    ESP.restart();
  }

  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
}

void loop() {
  float temperature = 25.0;
  
  if (temp_sensor != NULL) {
    esp_err_t result = temperature_sensor_get_celsius(temp_sensor, &temperature);
  }
  
  zbTempSensor.setTemperature(temperature);
  
  delay(5000);
}