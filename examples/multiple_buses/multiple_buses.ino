/**
 * @example Multiple Independent Buses for WirePIO
 * @brief
 * Demonstrates two independent I2C buses running simultaneously.
 * Each bus has its own PIO state machine and DMA channels,
 * operating completely independently.
 *
 * Wiring:
 *   Bus 1 (pins 2,3): Connect BME280 at 0x76
 *   Bus 2 (pins 4,5): Connect SSD1306 OLED at 0x3C
 *   (or any other I2C devices on each bus)
 *
 * This shows the key advantage of WirePIO over hardware I2C:
 * unlimited independent buses without multiplexers.
 */

#include <Arduino.h>
#include "WirePIO.h"

// Bus 1: Environmental sensors
WirePIO bus1(2, 3);

// Bus 2: Display
WirePIO bus2(4, 5);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    Serial.println("WirePIO Multiple Buses");
    Serial.println("======================");

    Serial.print("Bus 1 (pins 2,3): ");
    bus1.begin();
    if (bus1.scan() == 0) {
        Serial.println("  No devices found!");
    }

    Serial.print("Bus 2 (pins 4,5): ");
    bus2.begin();
    if (bus2.scan() == 0) {
        Serial.println("  No devices found!");
    }

    Serial.println("\nBoth buses are now running independently.");
    Serial.println("Use bus1 and bus2 like regular Wire objects:");
    Serial.println("  bus1.beginTransmission(0x76);");
    Serial.println("  bus1.write(0xF7);");
    Serial.println("  bus1.endTransmission();");
}

void loop() {
    delay(10000);
}
