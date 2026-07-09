/**
 * @file WirePIOSlave.h
 * @brief I2C Slave mode handler using RP2040 hardware I2C peripheral.
 *
 * Uses the RP2040's built-in I2C controller in slave mode for robust
 * handling of address matching, clock stretching, START/STOP detection,
 * and repeated start conditions. This is a proven, well-tested approach
 * (same as Arduino-Pico TwoWire).
 *
 * In a future version (v2), slave mode may also be implemented in PIO
 * to free the hardware I2C peripheral and allow any pin pair.
 *
 * @author angeloINTJ
 * @license MIT
 */

#ifndef WIREPIOSLAVE_H
#define WIREPIOSLAVE_H

#include "WirePIO_config.h"
#include <hardware/i2c.h>

/**
 * @brief I2C slave handler using hardware I2C peripheral.
 *
 * Manages the RP2040's built-in I2C controller in slave mode,
 * including IRQ handling for RX_FULL, RD_REQ, START_DET, and
 * STOP_DET events. Buffers received data and invokes user
 * callbacks on receive/request events.
 */
class WirePIOSlave {
public:
    WirePIOSlave();
    ~WirePIOSlave();

    /**
     * @brief Initialize I2C slave mode on the given pins.
     * @param i2c     Hardware I2C instance (i2c0 or i2c1).
     * @param sda     GPIO pin for SDA.
     * @param scl     GPIO pin for SCL.
     * @param address 7-bit I2C address for this slave.
     * @return true if slave mode was configured successfully.
     */
    bool begin(i2c_inst_t *i2c, uint8_t sda, uint8_t scl, uint8_t address);

    /**
     * @brief Shut down slave mode and release resources.
     */
    void end();

    /// @brief Returns true if slave mode is active.
    bool isActive() const { return _active; }

    /**
     * @brief Register callback for when the master sends data.
     * @param callback Function(int numBytes) invoked on STOP or buffer full.
     */
    void onReceive(void(*callback)(int)) { _onReceiveCallback = callback; }

    /**
     * @brief Register callback for when the master requests data.
     * @param callback Function() invoked when master reads from this slave.
     */
    void onRequest(void(*callback)(void)) { _onRequestCallback = callback; }

    /**
     * @brief Internal IRQ handler — called from I2C ISR.
     *
     * Processes I2C interrupt events: RX_FULL (data received),
     * RD_REQ (master wants data), START_DET, RESTART_DET, STOP_DET.
     */
    void onIRQ();

    /// @brief Get the slave buffer for direct access (used by onRequest).
    uint8_t *getBuffer() { return _slaveBuf; }
    size_t   getBufferSize() const { return _slaveBufSize; }
    volatile int &getBufferLen() { return _slaveBufLen; }

private:
    i2c_inst_t *_i2c;
    uint8_t     _addr;
    bool        _active;
    bool        _slaveStartDet;

    /// Slave data buffer (separate from main WirePIO buffer).
    static const size_t SLAVE_BUF_SIZE = 256;
    uint8_t  _slaveBuf[SLAVE_BUF_SIZE];
    size_t   _slaveBufSize;
    volatile int _slaveBufLen;
    volatile int _slaveBufOff;

    /// User callbacks.
    void (*_onReceiveCallback)(int);
    void (*_onRequestCallback)(void);

    /// Previous slave address (for recovery on error).
    uint8_t _prevAddr;
};

#endif // WIREPIOSLAVE_H
