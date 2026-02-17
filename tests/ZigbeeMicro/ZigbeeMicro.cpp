#include "Zigbee.h"

// =============================================================================
// ZigbeeMicro - Minimal Zigbee heartbeat test device
//
// Joins network as "ZigbeeMicro" end device, then every 10s reports:
//   EP1 (Heartbeat):    counter % 10
//   EP2 (Send Timing):  how long the previous send took (ms)
// =============================================================================

static const uint8_t EP_HEARTBEAT = 1;
static const uint8_t EP_TIMING    = 2;

ZigbeeAnalog zbHeartbeat(EP_HEARTBEAT);
ZigbeeAnalog zbTiming(EP_TIMING);

uint32_t counter            = 0;
uint32_t lastSendDurationMs = 0;

void setup() {
    Serial.begin(115200);
    delay(5000);  // allow time for flashing / serial connect
    Serial.println("=== ZigbeeMicro Test Device ===");

    // -- Heartbeat endpoint (counter % 10, range 0-9) --
    zbHeartbeat.setManufacturerAndModel("Espressif", "ZigbeeMicro");
    zbHeartbeat.addAnalogInput();
    zbHeartbeat.setAnalogInputDescription("Heartbeat");
    zbHeartbeat.setAnalogInputResolution(1);
    zbHeartbeat.setAnalogInputMinMax(0, 9);

    // -- Timing endpoint (last send duration in ms) --
    zbTiming.addAnalogInput();
    zbTiming.setAnalogInputDescription("Send Duration ms");
    zbTiming.setAnalogInputResolution(1);
    zbTiming.setAnalogInputMinMax(0, 60000);

    Zigbee.addEndpoint(&zbHeartbeat);
    Zigbee.addEndpoint(&zbTiming);

    // Start as Zigbee End Device
    esp_zb_cfg_t cfg = ZIGBEE_DEFAULT_ED_CONFIG();
    cfg.nwk_cfg.zed_cfg.keep_alive = 10000;
    Zigbee.setTimeout(10000);

    if (!Zigbee.begin(&cfg, false)) {
        Serial.println("Zigbee failed to start – rebooting...");
        ESP.restart();
    }

    Serial.print("Joining network");
    while (!Zigbee.connected()) {
        Serial.print(".");
        delay(100);
    }
    Serial.println(" connected!");
}

void loop() {
    float heartbeat = (float)(counter % 10);
    float timing    = (float)lastSendDurationMs;

    Serial.printf("[%lu] heartbeat=%d  lastSendMs=%lu\n",
                  counter, (int)heartbeat, lastSendDurationMs);

    uint32_t t0 = millis();

    zbHeartbeat.setAnalogInput(heartbeat);
    zbTiming.setAnalogInput(timing);
    zbHeartbeat.reportAnalogInput();
    zbTiming.reportAnalogInput();

    lastSendDurationMs = millis() - t0;

    Serial.printf("       send took %lu ms\n", lastSendDurationMs);

    counter++;
    delay(10000);
}
