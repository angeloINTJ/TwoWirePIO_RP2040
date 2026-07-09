/**
 * @example Master Echo (Write-Then-Read) for WirePIO
 * @brief
 * Demonstrates a write-then-read transaction using the TwoWire API.
 * Writes a register address, then reads back data from the device.
 *
 * Wiring:
 *   Connect a BME280 sensor (or any I2C device with readable registers):
 *   SDA → GPIO2, SCL → GPIO3, VCC → 3.3V, GND → GND
 *
 * Expected output (Serial Monitor, 115200 baud):
 *   WirePIO Master Echo
 *   ===================
 *   Chip ID: 0x60
 */

#include <Arduino.h>
#include "WirePIO.h"

#define SDA_PIN 2
#define SCL_PIN 3
#define BME280_ADDR 0x76
#define BME280_CHIP_ID_REG 0xD0

WirePIO bus(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    Serial.println("WirePIO Master Echo");
    Serial.println("===================");

    bus.begin();

    // Write register address, then read 1 byte
    bus.beginTransmission(BME280_ADDR);
    bus.write(BME280_CHIP_ID_REG);
    if (bus.endTransmission(false) != 0) {  // false = repeated start
        Serial.println("ERROR: Device not found at 0x76!");
        while (1) delay(1000);
    }

    bus.requestFrom(BME280_ADDR, 1);
    if (bus.available()) {
        uint8_t chipId = bus.read();
        Serial.print("Chip ID: 0x");
        Serial.println(chipId, HEX);
    } else {
        Serial.println("ERROR: No data received!");
    }
}

void loop() {
    delay(5000);
}
