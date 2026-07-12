/**
 * @file hardware_test.ino
 * @brief WirePIO hardware test example for RP2040.
 *
 * Tests the WirePIO library on actual hardware:
 * 1. I2C scanner on GPIO 2,3 (PIO+DMA)
 * 2. If a BME280/BMP280 is detected at 0x76, read its Chip ID
 * 3. Write-then-read transaction test
 *
 * Wiring for BME280 test:
 *   VCC → 3.3V, GND → GND, SDA → GPIO2, SCL → GPIO3
 *
 * Output: Serial Monitor at 115200 baud.
 */

#include <Arduino.h>
#include <WirePIO.h>

#define SDA_PIN 2
#define SCL_PIN 3

WirePIO bus(SDA_PIN, SCL_PIN, WIREPIO_FREQ_STANDARD);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    delay(500);

    Serial.println();
    Serial.println("=== WirePIO Hardware Test ===");
    Serial.println();

    // ── Test 1: begin() with PIO+DMA ───────────────────────────
    Serial.print("[1] Initializing WirePIO on GPIO");
    Serial.print(SDA_PIN);
    Serial.print("/");
    Serial.print(SCL_PIN);
    Serial.println("...");

    bus.begin();
    Serial.println("    begin() OK");
    Serial.print("    Clock: ");
    Serial.print(bus.getClock());
    Serial.println(" Hz");
    Serial.print("    Buffer: ");
    Serial.print(bus.available() == 0 ? "ready" : "data?");
    Serial.println();

    // ── Test 2: I2C Scanner ────────────────────────────────────
    Serial.println("[2] Scanning I2C bus...");
    int found = bus.scan();

    if (found == 0) {
        Serial.println("    WARNING: No I2C devices found!");
        Serial.println("    Check: VCC→3.3V GND→GND SDA→GPIO2 SCL→GPIO3");
        Serial.println("    External pull-up resistors (2.2k-4.7k) required on SDA!");
    }

    // ── Test 3: BME280 Chip ID read ────────────────────────────
    Serial.println("[3] Testing register read (write-then-read)...");

    // Write register address
    bus.beginTransmission(0x76);
    bus.write(0xD0);  // Chip ID register
    uint8_t txResult = bus.endTransmission(false);  // false = repeated start

    if (txResult != 0) {
        Serial.print("    No device at 0x76 (error code: ");
        Serial.print(txResult);
        Serial.println(")");
        if (txResult == 2) Serial.println("    → NACK on address — device not present");
        if (txResult == 3) Serial.println("    → NACK on data — register not writable");
        if (txResult == 5) Serial.println("    → Timeout — check wiring/pull-ups");
    } else {
        // Read Chip ID
        size_t n = bus.requestFrom(0x76, 1);
        if (n > 0 && bus.available()) {
            uint8_t chipId = bus.read();
            Serial.print("    Chip ID: 0x");
            Serial.print(chipId, HEX);

            if (chipId == 0x58) {
                Serial.println(" → BMP280 detected");
                Serial.println("    WIREPIO WORKS!");
            } else if (chipId == 0x60) {
                Serial.println(" → BME280 detected");
                Serial.println("    WIREPIO WORKS!");
            } else {
                Serial.print(" → Unknown (expected 0x58 or 0x60)");
                Serial.println();
            }
        } else {
            Serial.println("    ERROR: No data received from 0x76");
        }
    }

    // ── Test 4: end() + re-begin() cycle ────────────────────────
    Serial.println("[4] Testing end() / begin() cycle...");
    bus.end();
    Serial.println("    end() OK");
    delay(100);
    bus.begin();
    Serial.println("    re-begin() OK");

    Serial.println();
    Serial.println("=== Test Complete ===");
    Serial.println("WirePIO is ready for use.");
    Serial.println("Try: bus.beginTransmission(addr); bus.write(reg); bus.endTransmission();");
    Serial.println();
}

void loop() {
    // Periodic scan to show the bus is alive
    static unsigned long lastScan = 0;
    if (millis() - lastScan > 10000) {
        lastScan = millis();
        Serial.print("[scan] ");
        bus.scan();
    }
    delay(1000);
}
