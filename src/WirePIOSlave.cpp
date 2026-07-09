/**
 * @file WirePIOSlave.cpp
 * @brief I2C Slave mode handler — RP2040 hardware I2C peripheral.
 *
 * Uses the RP2040's built-in I2C controller as a slave device,
 * following the same register-access pattern as Arduino-Pico's
 * TwoWire for proven reliability.
 *
 * Interrupt events:
 *   bit 2  = RX_FULL      — data byte received from master
 *   bit 5  = RD_REQ       — master wants to read (onRequest)
 *   bit 6  = TX_ABRT      — transaction aborted
 *   bit 9  = STOP_DET     — STOP condition (end of transaction)
 *   bit 10 = START_DET    — START condition (new transaction)
 *   bit 12 = RESTART_DET  — repeated START (fires onReceive if buffered)
 *
 * @author angeloINTJ
 * @license MIT
 */

#include "WirePIOSlave.h"
#include <hardware/gpio.h>
#include <hardware/irq.h>

// ─── Static pointer for IRQ handler dispatch ─────────────────────────

// TwoWire in arduino-pico uses a static _handler0/_handler1 pattern.
// WirePIOSlave similarly stores a pointer per I2C instance for dispatch.
// For v1 with a single slave, we use a simple static pointer.
// Multi-slave support (v2) would use an array indexed by i2c_hw_index().

static WirePIOSlave *_activeSlave = nullptr;

static void _slaveIRQHandler() {
    if (_activeSlave) {
        _activeSlave->onIRQ();
    }
}

// =========================================================================
// CONSTRUCTION / DESTRUCTION
// =========================================================================

WirePIOSlave::WirePIOSlave()
    : _i2c(nullptr), _addr(0), _active(false), _slaveStartDet(false),
      _slaveBufSize(SLAVE_BUF_SIZE), _slaveBufLen(0), _slaveBufOff(0),
      _onReceiveCallback(nullptr), _onRequestCallback(nullptr), _prevAddr(0)
{}

WirePIOSlave::~WirePIOSlave() {
    end();
}

// =========================================================================
// BEGIN / END
// =========================================================================

bool WirePIOSlave::begin(i2c_inst_t *i2c, uint8_t sda, uint8_t scl, uint8_t address) {
    if (_active) end();

    _i2c = i2c;
    _addr = address;
    _prevAddr = address;
    _slaveBufLen = 0;
    _slaveBufOff = 0;
    _slaveStartDet = false;

    // Initialize I2C hardware as slave
    i2c_init(_i2c, 100000);
    i2c_set_slave_mode(_i2c, true, _addr);

    // Enable interrupts: RX_FULL(2), RD_REQ(5), TX_ABRT(6),
    // STOP_DET(9), START_DET(10), RESTART_DET(12)
    _i2c->hw->intr_mask = (1 << 12) | (1 << 10) | (1 << 9)
                        | (1 << 6)  | (1 << 5)  | (1 << 2);

    // Install IRQ handler
    int irqNo = I2C0_IRQ + i2c_hw_index(_i2c);
    irq_set_exclusive_handler(irqNo, _slaveIRQHandler);
    irq_set_enabled(irqNo, true);

    // Configure GPIO pins for I2C function
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(scl);

    _activeSlave = this;
    _active = true;
    return true;
}

void WirePIOSlave::end() {
    if (!_active) return;

    // Remove IRQ handler
    int irqNo = I2C0_IRQ + i2c_hw_index(_i2c);
    irq_remove_handler(irqNo, _slaveIRQHandler);
    irq_set_enabled(irqNo, false);

    // Disable slave mode
    if (_i2c) {
        _i2c->hw->intr_mask = 0;
        i2c_set_slave_mode(_i2c, false, 0);
        i2c_deinit(_i2c);
    }

    _activeSlave = nullptr;
    _i2c = nullptr;
    _active = false;
}

// =========================================================================
// IRQ HANDLER
// =========================================================================

// Prevent compiler optimization in the IRQ handler to avoid race
// conditions (same fix as arduino-pico Wire.cpp, GitHub issue #979)
#pragma GCC push_options
#pragma GCC optimize ("O0")

void WirePIOSlave::onIRQ() {
    if (!_active || !_i2c) return;

    // Snapshot IRQ status to avoid races if it changes mid-handler
    uint32_t irqstat = _i2c->hw->intr_stat;
    if (irqstat == 0) return;

    // ── RX_FULL (bit 2): Master sent a data byte ─────────────────
    if (irqstat & (1 << 2)) {
        if (_slaveBufLen < (int)_slaveBufSize) {
            _slaveBuf[_slaveBufLen++] = _i2c->hw->data_cmd & 0xFF;
        } else {
            // Buffer full — read and discard
            (void)_i2c->hw->data_cmd;
        }
    }

    // ── RD_REQ (bit 5): Master wants to read ─────────────────────
    if (irqstat & (1 << 5)) {
        if (_onRequestCallback) {
            _onRequestCallback();
        }
        _i2c->hw->clr_rd_req;
    }

    // ── TX_ABRT (bit 6): Transaction aborted ─────────────────────
    if (irqstat & (1 << 6)) {
        _i2c->hw->clr_tx_abrt;
    }

    // ── START_DET (bit 10): START condition ──────────────────────
    if (irqstat & (1 << 10)) {
        _slaveStartDet = true;
        _i2c->hw->clr_start_det;
    }

    // ── RESTART_DET (bit 12): Repeated START ─────────────────────
    if (irqstat & (1 << 12)) {
        if (_onReceiveCallback && _slaveBufLen) {
            _onReceiveCallback(_slaveBufLen);
        }
        _slaveBufLen = 0;
        _slaveBufOff = 0;
        _slaveStartDet = false;
        _i2c->hw->clr_restart_det;
    }

    // ── STOP_DET (bit 9): End of transaction ─────────────────────
    if (irqstat & (1 << 9)) {
        if (_onReceiveCallback && _slaveBufLen) {
            _onReceiveCallback(_slaveBufLen);
        }
        _slaveBufLen = 0;
        _slaveBufOff = 0;
        _slaveStartDet = false;
        _i2c->hw->clr_stop_det;
    }
}

#pragma GCC pop_options
