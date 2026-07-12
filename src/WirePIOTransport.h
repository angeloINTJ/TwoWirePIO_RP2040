/**
 * @file WirePIOTransport.h
 * @brief I2C Master transport layer — GPIO bit-bang and PIO+DMA burst.
 *
 * Two operating modes:
 * - **GPIO mode**: Blocking bit-bang on any pin pair using open-drain
 *   emulation via direction toggling.
 * - **PIO+DMA mode**: The PIO state machine executes I2C bursts autonomously.
 *   DMA channels feed commands and collect responses with zero CPU overhead
 *   during transfer.
 *
 * Based on PIO_I2C from BMx280PIO_RP2040, extended for TwoWire API needs:
 * - Variable-length writes and reads (up to buffer size)
 * - NACK detection with error propagation
 * - Timeout support on all blocking operations
 * - Async DMA with IRQ callback
 *
 * @author angeloINTJ
 * @license MIT
 */

#ifndef WIREPIOTRANSPORT_H
#define WIREPIOTRANSPORT_H

#include "WirePIO_config.h"

/**
 * @brief I2C master transport — GPIO bit-bang and PIO+DMA burst.
 *
 * Owns GPIO pins and PIO state machine resources. Methods are designed
 * to be called from WirePIO's TwoWire-compatible public API.
 */
class WirePIOTransport {
public:
    /**
     * @brief Construct an I2C master on the given pins.
     * @param sda GPIO pin for SDA (open-drain emulated via direction toggle).
     * @param scl GPIO pin for SCL.
     * @param freq Bus frequency in Hz (default 100 kHz).
     */
    WirePIOTransport(uint8_t sda, uint8_t scl, uint32_t freq = WIREPIO_DEFAULT_FREQ);
    ~WirePIOTransport();

    /// @name Lifecycle
    /// @{

    /**
     * @brief Initialize GPIO pins for bit-bang I2C.
     * @return true on success.
     */
    bool beginGPIO();

    /**
     * @brief Load PIO program and claim DMA channels.
     *
     * After this call, masterWrite() and masterRead() use PIO+DMA
     * for zero-CPU-overhead transfers.
     *
     * @param pio PIO instance (pio0 or pio1).
     * @return true if PIO program loaded and DMA channels claimed.
     */
    bool beginPIO(PIO pio = pio0);

    /**
     * @brief Release all resources (GPIO, PIO, DMA).
     */
    void end();

    bool isInitialized() const { return _gpioReady; }
    bool isPIOActive()    const { return _pioReady; }
    /// @}

    /// @name GPIO Bit-Bang Operations
    /// @{

    /**
     * @brief Write data to an I2C device via GPIO bit-bang.
     * @param addr   7-bit I2C address.
     * @param data   Transmit buffer.
     * @param len    Number of bytes to send.
     * @param stop   If true, generate STOP at end (false = repeated start).
     * @return WIREPIO_SUCCESS, WIREPIO_ERR_NACK_ADDR, or WIREPIO_ERR_NACK_DATA.
     */
    uint8_t gpioWrite(uint8_t addr, const uint8_t *data, size_t len, bool stop = true);

    /**
     * @brief Read data from an I2C device via GPIO bit-bang.
     * @param addr   7-bit I2C address.
     * @param data   Receive buffer.
     * @param len    Number of bytes to read.
     * @param stop   If true, generate STOP at end.
     * @return Number of bytes actually read (0 on NACK).
     */
    size_t gpioRead(uint8_t addr, uint8_t *data, size_t len, bool stop = true);

    /**
     * @brief Probe a device address (zero-byte write, check ACK).
     * @param addr 7-bit I2C address.
     * @return WIREPIO_SUCCESS if ACK, WIREPIO_ERR_NACK_ADDR if NACK.
     */
    uint8_t probeAddress(uint8_t addr);

    /**
     * @brief Scan the I2C bus and print found addresses.
     *
     * Outputs via Serial (Arduino) or printf (Pico SDK).
     * @return Number of devices found.
     */
    int scan();
    /// @}

    /// @name PIO+DMA Burst Operations
    /// @{

    /**
     * @brief DMA-driven write transaction.
     *
     * Builds command words from the buffer, feeds them via DMA to the
     * PIO TX FIFO, and waits for completion. NACK is detected via
     * PIO IRQ 0.
     *
     * @param addr   7-bit I2C address.
     * @param data   Transmit buffer.
     * @param len    Number of bytes to send.
     * @param stop   Generate STOP at end.
     * @return WIREPIO_SUCCESS, WIREPIO_ERR_NACK_ADDR, or WIREPIO_ERR_NACK_DATA.
     */
    uint8_t pioWrite(uint8_t addr, const uint8_t *data, size_t len, bool stop = true);

    /**
     * @brief DMA-driven read transaction.
     *
     * Feeds read-command words via DMA, collects received bytes from
     * the PIO RX FIFO via DMA. First RX word (address echo) is discarded.
     *
     * @param addr   7-bit I2C address.
     * @param data   Receive buffer.
     * @param len    Number of bytes to read.
     * @param stop   Generate STOP at end.
     * @return Number of bytes read (0 on error).
     */
    size_t pioRead(uint8_t addr, uint8_t *data, size_t len, bool stop = true);

    /**
     * @brief DMA-driven write-then-read (combined transaction with repeated start).
     * @param addr    7-bit I2C address.
     * @param wdata   Write buffer (register address, etc.)
     * @param wlen    Write length.
     * @param rdata   Read buffer.
     * @param rlen    Read length.
     * @return Number of bytes read (0 on error).
     */
    size_t burstRead(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

    size_t pioWriteThenRead(uint8_t addr,
                            const uint8_t *wdata, size_t wlen,
                            uint8_t *rdata, size_t rlen);
    /// @}

    /// @name GPIO Pin Access
    /// @{
    uint8_t getSDA() const { return _sda; }
    uint8_t getSCL() const { return _scl; }
    uint32_t getFrequency() const { return _freq; }
    /// @}

    // ─── Error counters ──────────────────────────────────────────────
    uint32_t nackCount = 0;
    uint32_t timeoutCount = 0;
    uint32_t readCount = 0;
    uint32_t writeCount = 0;
    void resetStats() { nackCount = timeoutCount = readCount = writeCount = 0; }

private:
    uint8_t  _sda, _scl;                    ///< GPIO pin numbers.
    uint32_t _freq;                         ///< I2C bus frequency in Hz.
    PIO      _pio;                          ///< PIO instance (pio0 or pio1).
    int      _sm, _offset;                  ///< PIO state machine and program offset.
    bool     _gpioReady;                    ///< GPIO pins initialized.
    bool     _pioReady;                     ///< PIO program loaded and DMA claimed.

    int      _dmaTx;                        ///< TX DMA channel (commands → PIO).
    int      _dmaRx;                        ///< RX DMA channel (PIO → buffer).


    /// Command buffer for TX DMA (built before each transaction).
    uint32_t _cmdBuf[WIREPIO_MAX_CMD_WORDS];
    size_t   _cmdCount;

    /// @name Internal Helpers — GPIO
    /// @{
    void _gpioStart();
    void _gpioStop();
    bool _gpioWriteByte(uint8_t data);      ///< Returns true if ACK received.
    uint8_t _gpioReadByte(bool last);       ///< Reads byte, sends ACK/NACK.
    void _gpioDelay();                      ///< Half-bit delay.
    /// @}

    /// @name Internal Helpers — PIO+DMA
    /// @{
    void _buildWriteCommands(uint8_t addr, const uint8_t *data, size_t len, bool stop);
    void _buildReadCommands(uint8_t addr, size_t len, bool stop);
    void _buildWriteThenReadCommands(uint8_t addr,
                                     const uint8_t *wdata, size_t wlen,
                                     size_t rlen);
    bool _waitDMADone(uint32_t timeoutUs);
    void _pioToGPIO();                      ///< Restore GPIO SIO mode from PIO.
    /// @}
};

#endif // WIREPIOTRANSPORT_H
