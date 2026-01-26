#include <Arduino.h>

#include "../include/config.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.println("Hello World!");
  delay(1000);
}
