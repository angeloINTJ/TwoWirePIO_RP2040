/**
 * @file WirePIO.h
 * @brief TwoWire-compatible I2C implementation using PIO+DMA on the RP2040.
 *
 * WirePIO provides the same interface as Arduino's TwoWire (Wire.h) but uses
 * the RP2040's PIO (Programmable I/O) and DMA to create I2C buses on any
 * GPIO pin pair — no hardware I2C peripheral required (for master mode).
 *
 * Key features:
 * - Drop-in replacement for Wire: WirePIO bus(2, 3); bus.begin();
 * - Multiple independent buses on any GPIO pins
 * - PIO+DMA burst transfers with zero CPU overhead during transfer
 * - GPIO bit-bang fallback for configuration/setup
 * - Slave mode via hardware I2C peripheral
 * - Async DMA transfers with callback
 * - Timeout and NACK detection
 *
 * @author angeloINTJ
 * @license MIT
 */

#ifndef WIREPIO_H
#define WIREPIO_H

#include "WirePIO_config.h"

#ifdef WIREPIO_PLATFORM_ARDUINO
  #include "api/HardwareI2C.h"
#endif

#include "WirePIOTransport.h"

// Forward declaration for slave handler
class WirePIOSlave;

// Feature detection macros (for library compatibility checks)
#define WIREPIO_HAS_END             1
#define WIREPIO_HAS_BUFFER_SIZE     1
#define WIREPIO_HAS_TIMEOUT         1
#define WIREPIO_HAS_BURST_READ      1
#define WIREPIO_HAS_SET_PIO         1
#define WIREPIO_HAS_CLOCK_STRETCH   1

/// @name Bus Initialization Modes
/// @{
#define WIREPIO_MODE_DEFAULT        0   ///< PIO+DMA with GPIO fallback
#define WIREPIO_MODE_GPIO_ONLY      1   ///< GPIO bit-bang only (no PIO/DMA)
/// @}

/**
 * @brief TwoWire-compatible I2C bus using PIO+DMA on the RP2040.
 *
 * Usage:
 * @code
 *   WirePIO bus1(2, 3);        // SDA=GPIO2, SCL=GPIO3
 *   bus1.begin();               // Master mode, enables PIO+DMA
 *   bus1.beginTransmission(0x76);
 *   bus1.write(0xF7);           // Register address
 *   bus1.endTransmission(false); // Repeated start
 *   bus1.requestFrom(0x76, 8);  // Read 8 bytes
 *   while (bus1.available()) { uint8_t b = bus1.read(); }
 * @endcode
 *
 * On Arduino-Pico, WirePIO extends arduino::HardwareI2C for maximum
 * compatibility with libraries expecting a TwoWire reference.
 * On Pico SDK, it is a standalone class with the same API.
 */
#ifdef WIREPIO_PLATFORM_ARDUINO
class WirePIO : public arduino::HardwareI2C {
#else
class WirePIO {
#endif
public:
    // ─── Construction ────────────────────────────────────────────────

    /**
     * @brief Construct an I2C bus on the given pins.
     * @param sda  GPIO pin for SDA (open-drain, external pull-up required).
     * @param scl  GPIO pin for SCL (driven by PIO/side-set, push-pull).
     * @param freq Bus frequency in Hz (default 100 kHz).
     * @param pio  PIO block to use (pio0 or pio1, default pio0).
     */
    WirePIO(pin_size_t sda, pin_size_t scl, uint32_t freq = WIREPIO_DEFAULT_FREQ,
            PIO pio = pio0);
    ~WirePIO();

    // ─── Pin Configuration (call before begin()) ─────────────────────

    /**
     * @brief Change SDA pin. Must be called before begin().
     * @return true if pin can be changed (bus not running).
     */
    bool setSDA(pin_size_t sda);

    /**
     * @brief Change SCL pin. Must be called before begin().
     * @return true if pin can be changed (bus not running).
     */
    bool setSCL(pin_size_t scl);

    /**
     * @brief Set the PIO block to use for I2C master mode.
     *
     * Must be called before begin(). Use this to avoid resource conflicts
     * with other PIO-based libraries (e.g., pio1 when WiFi is using pio0
     * on a Pico W, or when BMx280PIO_RP2040 occupies the other block).
     *
     * @param pio PIO instance (pio0 or pio1). Default is pio0.
     * @return true if the PIO block can be changed (bus not running).
     */
    bool setPIO(PIO pio);

    // ─── Lifecycle ────────────────────────────────────────────────────

    /**
     * @brief Initialize the bus as I2C master.
     *
     * Initializes GPIO, loads the PIO program, claims a state machine
     * and 2 DMA channels. After this call, the bus is ready for
     * beginTransmission() and requestFrom().
     *
     * Uses the PIO block configured via setPIO() or the constructor
     * (default: pio0).
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void begin() override;
#else
    void begin();
#endif

    /**
     * @brief Initialize the bus as I2C master on a specific PIO block.
     *
     * Overload that sets the PIO block before initializing. Equivalent
     * to calling setPIO(pio) followed by begin().
     *
     * @param pio PIO instance (pio0 or pio1).
     */
    void begin(PIO pio);

    /**
     * @brief Initialize the bus with specific mode.
     *
     * @param mode WIREPIO_MODE_DEFAULT (PIO+DMA) or WIREPIO_MODE_GPIO_ONLY.
     *             GPIO-only saves 1 PIO SM + 2 DMA channels for resource-
     *             constrained setups (e.g. Pico W with WiFi + multiple buses).
     */
    void begin(int mode);

    /**
     * @brief Initialize the bus as I2C slave with the given address.
     *
     * Uses the RP2040's hardware I2C peripheral for robust slave operation.
     * The GPIO pins must support I2C function for the chosen peripheral.
     *
     * @param address 7-bit I2C address for this slave device.
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void begin(uint8_t address) override;
#else
    void begin(uint8_t address);
#endif

    /**
     * @brief Shut down the bus and release all resources.
     *
     * Frees PIO state machine, DMA channels, GPIO pins, and buffers.
     * begin() can be called again after end().
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void end() override;
#else
    void end();
#endif

    /// @brief Returns true if the bus is initialized (master or slave).
    bool isRunning() const { return _running; }

    // ─── Clock ────────────────────────────────────────────────────────

    /**
     * @brief Set the I2C clock frequency.
     *
     * For master mode, this changes the internal frequency variable.
     * Re-initialize the bus (end() + begin()) for the change to take
     * effect on the PIO clock divider.
     *
     * @param freqHz Frequency in Hz (e.g., 100000 or 400000).
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void setClock(uint32_t freqHz) override;
#else
    void setClock(uint32_t freqHz);
#endif

    /// @brief Get the current clock frequency.
    uint32_t getClock() const { return _freq; }

    // ─── Buffer Size ──────────────────────────────────────────────────

    /**
     * @brief Set the internal TX/RX buffer size.
     *
     * Must be called before begin(). Minimum is WIREPIO_BUFFER_SIZE_MIN (32).
     * Default is WIREPIO_BUFFER_SIZE (256).
     *
     * @param bSize Desired buffer size in bytes.
     * @return The actual buffer size set.
     */
    size_t setBufferSize(size_t bSize);

    // ─── Master Transmit ──────────────────────────────────────────────

    /**
     * @brief Begin a transmission to a slave device.
     *
     * Resets the TX buffer and sets the target address. Subsequent write()
     * calls append data to the buffer. The data is sent on endTransmission().
     *
     * @param address 7-bit I2C address of the target device.
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void beginTransmission(uint8_t address) override;
#else
    void beginTransmission(uint8_t address);
#endif

    /**
     * @brief Send the buffered data and end the transmission.
     *
     * Uses PIO+DMA for zero-CPU-overhead transfer when the PIO is active,
     * falling back to GPIO bit-bang otherwise.
     *
     * @param stopBit If true (default), send STOP condition at end.
     *                If false, send repeated START (for write-then-read).
     * @return WIREPIO_SUCCESS (0) on success, or error code:
     *         1 = data too long, 2 = NACK on address, 3 = NACK on data,
     *         4 = other error, 5 = timeout.
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    uint8_t endTransmission(bool stopBit) override;
    uint8_t endTransmission(void) override;
#else
    uint8_t endTransmission(bool stopBit = true);
    uint8_t endTransmission(void) { return endTransmission(true); }
#endif

    // ─── Master Receive ───────────────────────────────────────────────

    /**
     * @brief Request bytes from a slave device.
     *
     * Reads data into the internal buffer. Use available() and read()
     * to consume the received bytes.
     *
     * @param address  7-bit I2C address.
     * @param quantity Number of bytes to request.
     * @param stopBit  If true (default), send STOP at end.
     * @return Number of bytes actually received (0 on NACK/error).
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    size_t requestFrom(uint8_t address, size_t quantity, bool stopBit) override;
        size_t requestFrom(uint8_t address, size_t quantity) override;

    /**
     * @brief Combined write-register-then-read in a single PIO+DMA burst.
     *
     * Sends a register address followed by a read in one PIO operation,
     * without releasing the bus between write and read. Uses the proven
     * burstRead pattern for BMP280/BME280 register access.
     *
     * @param address 7-bit I2C address.
     * @param reg     Register address to write.
     * @param data    Buffer for received bytes.
     * @param len     Number of bytes to read (max 8).
     * @return Number of bytes read (0 on error).
     */
    size_t burstRead(uint8_t address, uint8_t reg, uint8_t *data, size_t len);

    size_t writeThenRead(uint8_t address,
                         const uint8_t *wdata, size_t wlen,
                         uint8_t *rdata, size_t rlen,
                         bool stopBit = true);
#else
    size_t requestFrom(uint8_t address, size_t quantity, bool stopBit = true);
    size_t requestFrom(uint8_t address, size_t quantity);
#endif

    // ─── Stream Interface (RX consumption) ────────────────────────────

    /**
     * @brief Returns the number of bytes available to read from the RX buffer.
     */
    virtual int available(void);

    /**
     * @brief Read one byte from the RX buffer.
     * @return The byte (0-255), or -1 if no data available.
     */
    virtual int read(void);

    /**
     * @brief Peek at the next byte without consuming it.
     * @return The next byte, or -1 if no data available.
     */
    virtual int peek(void);

    /**
     * @brief Flush the transmit buffer (no-op; data is sent in endTransmission).
     */
    virtual void flush(void);

    // ─── Stream Write (TX buffering) ──────────────────────────────────

    /**
     * @brief Write a single byte to the TX buffer.
     * @param data Byte to append.
     * @return 1 on success, 0 if buffer is full.
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *data, size_t quantity) override;
#else
    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);
#endif

    // Convenience overloads (from TwoWire)
    inline size_t write(unsigned long n)       { return write((uint8_t)n); }
    inline size_t write(long n)                { return write((uint8_t)n); }
    inline size_t write(unsigned int n)        { return write((uint8_t)n); }
    inline size_t write(int n)                 { return write((uint8_t)n); }

#ifdef WIREPIO_PLATFORM_ARDUINO
    using Print::write;  // Pull in print/write methods from Print base
#endif

    // ─── Slave Callbacks ──────────────────────────────────────────────

    /**
     * @brief Register a callback for when the master sends data to this slave.
     * @param callback Function(int numBytes) called when data is received.
     */
#ifdef WIREPIO_PLATFORM_ARDUINO
    void onReceive(void(*callback)(int)) override;
    void onRequest(void(*callback)(void)) override;
#else
    void onReceive(void(*callback)(int));
    void onRequest(void(*callback)(void));
#endif

    // ─── Async DMA Transfers ──────────────────────────────────────────

    /**
     * @brief Perform an asynchronous write-then-read transaction.
     *
     * The transfer happens via DMA in the background. Call finishedAsync()
     * to check for completion, or register a callback with onFinishedAsync().
     * Do not modify the buffers until the transfer is complete.
     *
     * @param address  7-bit I2C address.
     * @param wbuffer  Write data buffer (must remain valid).
     * @param wbytes   Number of bytes to write.
     * @param rbuffer  Read data buffer (must remain valid).
     * @param rbytes   Number of bytes to read.
     * @param sendStop Send STOP at end (default true).
     * @return true if the async transfer was started.
     */
    bool writeReadAsync(uint8_t address, const void *wbuffer, size_t wbytes,
                        const void *rbuffer, size_t rbytes, bool sendStop = true);

    /**
     * @brief Perform an asynchronous write-only transaction.
     */
    bool writeAsync(uint8_t address, const void *buffer, size_t bytes, bool sendStop = true);

    /**
     * @brief Perform an asynchronous read-only transaction.
     */
    bool readAsync(uint8_t address, void *buffer, size_t bytes, bool sendStop = true);

    /// @brief Returns true if the hardware bus is idle (no async transfer running).
    bool busIdle();

    /// @brief Check if the async transfer has finished.
    bool finishedAsync();

    /// @brief Cancel an in-progress async transfer.
    void abortAsync();

    /// @brief Register a callback for async transfer completion.
    void onFinishedAsync(void(*callback)(void));

    /**
     * @brief Internal DMA IRQ handler. Public for low-level access.
     */
    void _dma_irq_handler();

    // ─── Timeout ──────────────────────────────────────────────────────

    /**
     * @brief Set the I2C timeout.
     * @param timeout Timeout in milliseconds (default 25).
     * @param reset_with_timeout If true, attempt bus recovery on timeout.
     */
    void setTimeout(uint32_t timeout = 25, bool reset_with_timeout = false);

    /// @brief Returns true if a timeout occurred.
    bool getTimeoutFlag(void);

    /// @brief Clear the timeout flag.
    void clearTimeoutFlag(void);

    // ─── Utility ──────────────────────────────────────────────────────

    /**
     * @brief Scan the I2C bus and print all responding addresses.
     * @return Number of devices found.
     */
    int scan();
    int scan(uint8_t *buf, size_t max);
    bool busRecovery();

private:
    pin_size_t _sda, _scl;                  ///< GPIO pin numbers.
    uint32_t   _freq;                       ///< Clock frequency.
    PIO        _pioBlock;                   ///< PIO block for master mode (pio0 or pio1).
    bool       _running;                     ///< Bus initialized (master or slave).
    bool       _slave;                      ///< True if slave mode.
    uint8_t    _addr;                       ///< Target/slave address.
    bool       _txBegun;                    ///< beginTransmission() was called.

    /// Shared TX/RX buffer (heap-allocated).
    uint8_t *_buff;
    size_t   _buffSize;
    int      _buffLen;                      ///< Valid bytes in buffer (TX count or RX available).
    int      _buffOff;                      ///< Read cursor for RX consumption.

    /// Transport layer (GPIO bit-bang + PIO+DMA).
    WirePIOTransport *_transport;

    /// Slave handler (hardware I2C peripheral).
    WirePIOSlave *_slaveHandler;

    // ─── Timeout ──────────────────────────────────────────────────
    uint32_t _timeoutMs;
    bool     _timeoutFlag;
    bool     _resetWithTimeout;
    void     _handleTimeout(bool reset);

    // ─── Callbacks ─────────────────────────────────────────────────
    void (*_onRequestCallback)(void);
    void (*_onReceiveCallback)(int);

    // ─── DMA / Async ───────────────────────────────────────────────
    bool     _dmaRunning;
    int      _dmaChannelSend;
    int      _dmaChannelReceive;
    uint16_t *_dmaSendBuffer;
    size_t   _dmaSendBufferLen;
    volatile bool _dmaFinished;
    void (*_dmaOnFinished)(void);
    void _beginAsync();
    void _endAsync();

    // ─── Internal Helpers ──────────────────────────────────────────
    void _allocateBuffer();
    void _freeBuffer();
    uint8_t _sendBuffer(bool stop);
};

#endif // WIREPIO_H
