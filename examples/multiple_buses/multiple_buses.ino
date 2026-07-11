/**
 * @example Multiple Independent Buses for WirePIO
 * @brief
 * Demonstrates two independent I2C buses running simultaneously.
 * Each bus has its own PIO state machine and DMA channels,
 * operating completely independently.
 *
 * PIO block assignment:
 *   - Bus 1: pio0 (default)
 *   - Bus 2: pio1 (explicit) — avoids instruction-space conflicts
 *     when sharing a PIO block with other libraries (BMx280PIO,
 *     CYW43 WiFi on Pico W, etc.)
 *
 * On Pico W, WiFi occupies pio1. Use pio0 for WirePIO buses:
 *   WirePIO bus(2, 3, 100000, pio0);
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

// Bus 1: Environmental sensors (pio0)
WirePIO bus1(2, 3);

// Bus 2: Display (pio1 — separate PIO block to avoid conflicts)
// Constructor: WirePIO(sda, scl, freq, pio)
WirePIO bus2(4, 5, WIREPIO_FREQ_STANDARD, pio1);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    Serial.println("WirePIO Multiple Buses");
    Serial.println("======================");

    Serial.print("Bus 1 (pins 2,3, pio0): ");
    bus1.begin();
    if (bus1.scan() == 0) {
        Serial.println("  No devices found!");
    }

    Serial.print("Bus 2 (pins 4,5, pio1): ");
    bus2.begin();
    if (bus2.scan() == 0) {
        Serial.println("  No devices found!");
    }

    // Alternative API — setPIO() before begin():
    // WirePIO bus3(6, 7);
    // bus3.setPIO(pio1);
    // bus3.begin();

    // Or use the begin(PIO) overload:
    // WirePIO bus4(8, 9);
    // bus4.begin(pio1);

    Serial.println("\nBoth buses are now running independently.");
    Serial.println("Use bus1 and bus2 like regular Wire objects:");
    Serial.println("  bus1.beginTransmission(0x76);");
    Serial.println("  bus1.write(0xF7);");
    Serial.println("  bus1.endTransmission();");
}

void loop() {
    delay(10000);
}
