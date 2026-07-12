/**
 * @example I2C Scanner for WirePIO
 * @brief
 * Scans the I2C bus and prints all responding device addresses.
 * Demonstrates the simplest WirePIO usage: begin() + probe address.
 *
 * Wiring:
 *   Connect any I2C device (e.g., BME280 at 0x76) to:
 *   SDA → GPIO2, SCL → GPIO3, VCC → 3.3V, GND → GND
 *
 * Expected output (Serial Monitor, 115200 baud):
 *   WirePIO I2C Scanner
 *   ====================
 *   0x76
 *   (1 devices)
 */

#include <Arduino.h>
#include <WirePIO.h>

#define SDA_PIN 2
#define SCL_PIN 3

WirePIO bus(SDA_PIN, SCL_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    Serial.println("WirePIO I2C Scanner");
    Serial.println("====================");

    bus.begin();
    bus.scan();
}

void loop() {
    delay(5000);
}
