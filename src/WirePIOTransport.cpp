/**
 * @file WirePIOTransport.cpp
 * @brief I2C master transport implementation — GPIO bit-bang + PIO+DMA burst.
 *
 * GPIO mode uses open-drain emulation via direction toggling on any pin pair.
 * PIO+DMA mode uses the i2c.pio state machine with manual DMA register writes
 * (workaround for SDK dma_channel_configure TX count bug on RP2040).
 *
 * Based on PIO_I2C.cpp from BMx280PIO_RP2040, extended with:
 * - NACK detection via PIO IRQ 0
 * - Clock stretching (wait 1 pin in PIO)
 * - Timeout on all blocking operations
 * - Variable-length transactions (up to buffer size)
 *
 * @author angeloINTJ
 * @license MIT
 */

#include "WirePIOTransport.h"

#ifdef WIREPIO_PLATFORM_ARDUINO
  #include <hardware/gpio.h>
  #include <hardware/clocks.h>
  #include <hardware/dma.h>
#endif

#include "i2c.pio.h"

// =========================================================================
// STATIC HELPERS
// =========================================================================

/// 8-bit bit-reversal (used for PIO command encoding).
static uint8_t rev8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

/// Build a 16-bit PIO command word.
/// @param start  Generate START condition before this byte.
/// @param read   Read from slave (1) or write (0).
/// @param stop   Generate STOP after this byte.
/// @param data   Data byte (ignored for reads — use 0xFF).
static inline uint16_t mk_cmd(bool start, bool read, bool stop, uint8_t data) {
    uint8_t inv = (~rev8(data)) & 0xFF;
    return (start ? 1u : 0u) |
           ((read  ? 1u : 0u) << 1) |
           (((uint16_t)inv) << 2) |
           ((stop  ? 1u : 0u) << 10);
}

/// GPIO open-drain helpers: SDA is never driven HIGH.
/// Instead, pin direction is switched to INPUT (Hi-Z), relying on
/// the external pull-up resistor.
static inline void sda_lo(uint p) { gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0); }
static inline void sda_hi(uint p) { gpio_set_dir(p, GPIO_IN); }
static inline bool sda_read(uint p) { return gpio_get(p); }
static inline void scl_lo(uint p) { gpio_put(p, 0); }
static inline void scl_hi(uint p) { gpio_put(p, 1); }

// =========================================================================
// CONSTRUCTION / DESTRUCTION
// =========================================================================

WirePIOTransport::WirePIOTransport(uint8_t sda, uint8_t scl, uint32_t freq)
    : _sda(sda), _scl(scl), _freq(freq), _pio(nullptr), _sm(-1), _offset(-1),
      _gpioReady(false), _pioReady(false), _dmaTx(-1), _dmaRx(-1), _cmdCount(0)
{}

WirePIOTransport::~WirePIOTransport() { end(); }

// =========================================================================
// GPIO LIFECYCLE
// =========================================================================

bool WirePIOTransport::beginGPIO() {
    if (_gpioReady) return true;
    gpio_init(_sda);
    gpio_set_dir(_sda, GPIO_IN);
    gpio_pull_up(_sda);
    gpio_init(_scl);
    gpio_set_dir(_scl, GPIO_OUT);
    gpio_put(_scl, 1);
    _gpioReady = true;
    return true;
}

// =========================================================================
// GPIO BIT-BANG HELPERS
// =========================================================================

void WirePIOTransport::_gpioDelay() {
    uint32_t half = 500000 / _freq;
    if (half < 2) half = 2;
    WIREPIO_DELAY_US(half);
}

void WirePIOTransport::_gpioStart() {
    sda_lo(_sda);
    _gpioDelay();
    scl_lo(_scl);
    _gpioDelay();
}

void WirePIOTransport::_gpioStop() {
    sda_lo(_sda);
    _gpioDelay();
    scl_hi(_scl);
    _gpioDelay();
    sda_hi(_sda);
    _gpioDelay();
}

bool WirePIOTransport::_gpioWriteByte(uint8_t data) {
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        if (data & mask) sda_hi(_sda); else sda_lo(_sda);
        _gpioDelay();
        scl_hi(_scl);
        _gpioDelay();
        scl_lo(_scl);
    }
    // Release SDA for ACK bit
    sda_hi(_sda);
    _gpioDelay();
    scl_hi(_scl);
    _gpioDelay();
    bool ack = !sda_read(_sda);  // ACK = SDA low
    scl_lo(_scl);
    _gpioDelay();
    return ack;
}

uint8_t WirePIOTransport::_gpioReadByte(bool last) {
    uint8_t data = 0;
    sda_hi(_sda);
    for (int i = 0; i < 8; i++) {
        scl_hi(_scl);
        _gpioDelay();
        data = (data << 1) | (sda_read(_sda) ? 1 : 0);
        scl_lo(_scl);
        _gpioDelay();
    }
    // Send ACK (0) or NACK (1)
    if (last) sda_hi(_sda); else sda_lo(_sda);
    _gpioDelay();
    scl_hi(_scl);
    _gpioDelay();
    scl_lo(_scl);
    _gpioDelay();
    sda_hi(_sda);
    return data;
}

// =========================================================================
// GPIO TRANSACTIONS
// =========================================================================

uint8_t WirePIOTransport::gpioWrite(uint8_t addr, const uint8_t *data, size_t len, bool stop) {
    if (!_gpioReady || len == 0) return WIREPIO_ERR_OTHER;

    _gpioStart();
    if (!_gpioWriteByte((uint8_t)(addr << 1))) {
        _gpioStop();
        return WIREPIO_ERR_NACK_ADDR;
    }
    for (size_t i = 0; i < len; i++) {
        if (!_gpioWriteByte(data[i])) {
            _gpioStop();
            return WIREPIO_ERR_NACK_DATA;
        }
    }
    if (stop) _gpioStop();
    return WIREPIO_SUCCESS;
}

size_t WirePIOTransport::gpioRead(uint8_t addr, uint8_t *data, size_t len, bool stop) {
    if (!_gpioReady || len == 0) return 0;

    _gpioStart();
    if (!_gpioWriteByte((uint8_t)((addr << 1) | 1))) {
        _gpioStop();
        return 0;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = _gpioReadByte(i == len - 1);
    }
    if (stop) _gpioStop();
    return len;
}

uint8_t WirePIOTransport::probeAddress(uint8_t addr) {
    if (!_gpioReady) return WIREPIO_ERR_OTHER;

    _gpioStart();
    bool ack = _gpioWriteByte((uint8_t)(addr << 1));
    _gpioStop();
    return ack ? WIREPIO_SUCCESS : WIREPIO_ERR_NACK_ADDR;
}

int WirePIOTransport::scan() {
    if (!_gpioReady) return 0;

#ifdef WIREPIO_PLATFORM_ARDUINO
#else
    printf("I2C Scan:\n");
#endif

    int found = 0;
    for (int addr = 1; addr < 0x78; addr++) {
        _gpioStart();
        bool ack = _gpioWriteByte((uint8_t)(addr << 1));
        _gpioStop();
        if (ack) {
#ifdef WIREPIO_PLATFORM_ARDUINO


#else
            printf("0x%02X ", addr);
#endif
            found++;
        }
    }

#ifdef WIREPIO_PLATFORM_ARDUINO


#else
    printf("(%d devices)\n", found);
#endif
    return found;
}

// =========================================================================
// PIO+DMA LIFECYCLE
// =========================================================================

// Static cache: offset of the i2c_master program in each PIO block.
// The earlephilhower PIO allocator doesn't detect duplicate programs,
// so the first instance loads normally and caches the offset; subsequent
// instances reuse it instead of failing on PICO_ERROR_INSUFFICIENT_RESOURCES.
static int _shared_i2c_offset_wp[2] = {-1, -1};

bool WirePIOTransport::beginPIO(PIO pio) {
    if (_pioReady) return true;
    if (!_gpioReady) return false;

    _pio = pio;
    uint pio_idx = pio_get_index(_pio);

    if (_shared_i2c_offset_wp[pio_idx] >= 0) {
        // Program already loaded by another instance — reuse its offset
        _offset = _shared_i2c_offset_wp[pio_idx];
    } else {
        // First instance on this PIO block — load the program normally
        _offset = pio_add_program(_pio, &i2c_master_program);
        if (_offset < 0) return false;
        _shared_i2c_offset_wp[pio_idx] = _offset;
    }

    // Claim a state machine
    int sm = pio_claim_unused_sm(_pio, false);
    if (sm < 0) {
        pio_remove_program(_pio, &i2c_master_program, _offset);
        _offset = -1;
        return false;
    }
    _sm = sm;

    // Configure SM with pin mappings and clock divider
    pio_sm_config c = i2c_master_program_get_default_config(_offset);
    sm_config_set_out_pins(&c, _sda, 1);
    sm_config_set_set_pins(&c, _sda, 1);
    sm_config_set_in_pins(&c, _sda);
    sm_config_set_sideset_pins(&c, _scl);
    sm_config_set_in_shift(&c, false, true, 8);  // MSB first, autopush at 8

    float div = (float)clock_get_hz(clk_sys) / ((float)_freq * 13.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(_pio, _sm, _offset, &c);
    pio_sm_set_pindirs_with_mask(_pio, _sm,
        (1u << _scl),                           // SCL output enable
        (1u << _sda) | (1u << _scl));           // pin directions

    // Claim 2 DMA channels (TX + RX)
    _dmaTx = dma_claim_unused_channel(false);
    _dmaRx = dma_claim_unused_channel(false);

    if (_dmaTx < 0 || _dmaRx < 0) {
        if (_dmaTx >= 0) { dma_channel_unclaim(_dmaTx); _dmaTx = -1; }
        if (_dmaRx >= 0) { dma_channel_unclaim(_dmaRx); _dmaRx = -1; }
        pio_sm_unclaim(_pio, _sm);
        pio_remove_program(_pio, &i2c_master_program, _offset);
        _offset = -1;
        return false;
    }

    _pioReady = true;
    return true;
}

void WirePIOTransport::end() {
    // Release DMA channels
    if (_pioReady) {
        if (_dmaTx >= 0) {
            dma_channel_abort(_dmaTx);
            dma_channel_unclaim(_dmaTx);
            _dmaTx = -1;
        }
        if (_dmaRx >= 0) {
            dma_channel_abort(_dmaRx);
            dma_channel_unclaim(_dmaRx);
            _dmaRx = -1;
        }
        // Release PIO resources
        if (_pio && _offset >= 0) {
            pio_sm_set_enabled(_pio, _sm, false);
            pio_sm_unclaim(_pio, _sm);
            pio_remove_program(_pio, &i2c_master_program, _offset);
        }
        _offset = -1;
        _sm = -1;
        _pioReady = false;
    }

    // Release GPIO
    if (_gpioReady) {
        gpio_set_dir(_sda, GPIO_IN);
        gpio_disable_pulls(_sda);
        gpio_set_dir(_scl, GPIO_IN);
        gpio_disable_pulls(_scl);
        _gpioReady = false;
    }
}

// =========================================================================
// PIO+DMA COMMAND BUILDERS
// =========================================================================

void WirePIOTransport::_buildWriteCommands(uint8_t addr, const uint8_t *data,
                                            size_t len, bool stop) {
    _cmdCount = 0;
    // Word 0: START + address (write direction)
    _cmdBuf[_cmdCount++] = mk_cmd(true, false, false, (uint8_t)(addr << 1));
    // Words 1..N-1: data bytes (no START, no STOP)
    for (size_t i = 0; i < len - 1; i++) {
        _cmdBuf[_cmdCount++] = mk_cmd(false, false, false, data[i]);
    }
    // Last word: data + optional STOP
    _cmdBuf[_cmdCount++] = mk_cmd(false, false, stop, data[len - 1]);
}

void WirePIOTransport::_buildReadCommands(uint8_t addr, size_t len, bool stop) {
    _cmdCount = 0;
    // Word 0: START + address (read direction) — uses write path, STOP at bit 10
    _cmdBuf[_cmdCount++] = mk_cmd(true, false, false, (uint8_t)((addr << 1) | 1));
    // Read data words: the PIO read path consumes Y(1)+X(1)+pindirs(1)=3 bits
    // before check_stop reads STOP. Shift STOP from bit 10 → bit 3.
    for (size_t i = 0; i < len - 1; i++) {
        _cmdBuf[_cmdCount++] = mk_cmd(false, true, false, 0xFF);  // STOP already 0
    }
    uint16_t last = mk_cmd(false, true, stop, 0xFF);
    if (stop) { last = (last & ~(1u << 10)) | (1u << 3); }  // move STOP bit
    _cmdBuf[_cmdCount++] = last;
}

void WirePIOTransport::_buildWriteThenReadCommands(uint8_t addr,
                                                    const uint8_t *wdata, size_t wlen,
                                                    size_t rlen) {
    _cmdCount = 0;
    // Write phase
    // Word 0: START + address (write direction)
    _cmdBuf[_cmdCount++] = mk_cmd(true, false, false, (uint8_t)(addr << 1));
    // Words 1..wlen: write data bytes
    for (size_t i = 0; i < wlen; i++) {
        // Last write byte: no STOP, repeated START on next command
        _cmdBuf[_cmdCount++] = mk_cmd(false, false, false, wdata[i]);
    }
    // Read phase
    // Repeated START + address (read direction)
    _cmdBuf[_cmdCount++] = mk_cmd(true, false, false, (uint8_t)((addr << 1) | 1));
    // Read bytes (PIO read path: STOP must be at bit 3, not bit 10)
    for (size_t i = 0; i < rlen - 1; i++) {
        _cmdBuf[_cmdCount++] = mk_cmd(false, true, false, 0xFF);
    }
    // Last read byte + STOP (shifted to bit 3 for read path alignment)
    uint16_t last_rd = mk_cmd(false, true, true, 0xFF);
    last_rd = (last_rd & ~(1u << 10)) | (1u << 3);  // move STOP bit 10→3
    _cmdBuf[_cmdCount++] = last_rd;
}

// =========================================================================
// PIO+DMA WAIT / GPIO RESTORE
// =========================================================================

bool WirePIOTransport::_waitDMADone(uint32_t timeoutUs) {
    uint64_t deadline = WIREPIO_TIMEOUT_US(timeoutUs);
    while (dma_channel_is_busy(_dmaTx)) {
        if (WIREPIO_TIME_REACHED(deadline)) {
            dma_channel_abort(_dmaTx);
            dma_channel_abort(_dmaRx);
            return false;
        }
    }
    if (_dmaRx >= 0) {
        while (dma_channel_is_busy(_dmaRx)) {
            if (WIREPIO_TIME_REACHED(deadline)) {
                dma_channel_abort(_dmaRx);
                return false;
            }
        }
    }
    return true;
}

void WirePIOTransport::_pioToGPIO() {
    // Restore GPIO pins from PIO function to SIO (GPIO) mode
    gpio_set_function(_scl, GPIO_FUNC_SIO);
    gpio_set_dir(_scl, GPIO_OUT);
    gpio_put(_scl, 1);
    WIREPIO_DELAY_US(5);
    gpio_put(_scl, 0);
    WIREPIO_DELAY_US(5);
    gpio_put(_scl, 1);

    gpio_set_function(_sda, GPIO_FUNC_SIO);
    gpio_set_dir(_sda, GPIO_IN);
    gpio_pull_up(_sda);
}

// =========================================================================
// PIO+DMA TRANSACTIONS
// =========================================================================

uint8_t WirePIOTransport::pioWrite(uint8_t addr, const uint8_t *data,
                                    size_t len, bool stop) {
    if (!_pioReady || _dmaTx < 0) return WIREPIO_ERR_OTHER;
    if (len == 0) return WIREPIO_SUCCESS;

    _buildWriteCommands(addr, data, len, stop);

    // ─── Configure TX DMA (manual register writes) ─────────────────
    // Workaround for dma_channel_configure TX count bug on RP2040.
    // Registers are written directly; DMA is enabled AFTER PIO SM starts
    // to ensure DREQ signals are active before transfer begins.

    dma_channel_abort(_dmaTx);

    dma_channel_hw_t *tx_hw = &dma_hw->ch[_dmaTx];
    tx_hw->read_addr  = (uint32_t)_cmdBuf;
    tx_hw->write_addr = (uint32_t)&_pio->txf[_sm];
    tx_hw->transfer_count = _cmdCount;
    tx_hw->ctrl_trig = DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS |
                       DMA_CH0_CTRL_TRIG_INCR_READ_BITS |
                       (pio_get_dreq(_pio, _sm, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);

    // Clear PIO interrupt flag (for NACK detection)
    pio_interrupt_clear(_pio, _sm);

    // ─── Switch GPIO to PIO function ──────────────────────────────
    gpio_set_function(_sda, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(_scl, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_pull_up(_sda);

    // ─── Start PIO SM, then enable DMA ────────────────────────────
    pio_sm_clear_fifos(_pio, _sm);
    pio_sm_restart(_pio, _sm);
    pio_sm_set_enabled(_pio, _sm, true);

    tx_hw->ctrl_trig |= DMA_CH0_CTRL_TRIG_EN_BITS;

    // ─── Wait for completion ──────────────────────────────────────
    bool ok = _waitDMADone(WIREPIO_DEFAULT_TIMEOUT_US);
    pio_sm_set_enabled(_pio, _sm, false);

    // Check for NACK (PIO raises IRQ 0 and halts on unexpected NACK)
    bool nacked = pio_interrupt_get(_pio, _sm);
    pio_interrupt_clear(_pio, _sm);

    // ─── Restore GPIO to SIO mode ─────────────────────────────────
    _pioToGPIO();

    if (!ok) return WIREPIO_ERR_TIMEOUT;
    if (nacked) return WIREPIO_ERR_NACK_DATA;
    return WIREPIO_SUCCESS;
}

size_t WirePIOTransport::pioRead(uint8_t addr, uint8_t *data, size_t len, bool stop) {
    if (!_pioReady || _dmaTx < 0 || _dmaRx < 0) return 0;
    if (len == 0) return 0;

    _buildReadCommands(addr, len, stop);

    // RX buffer: +1 for address echo (first byte is discarded)
    // Use a local buffer to avoid corrupting _cmdBuf during read
    uint32_t rxbuf[WIREPIO_BUFFER_SIZE];

    // ─── Configure TX DMA ─────────────────────────────────────────
    dma_channel_abort(_dmaTx);

    dma_channel_hw_t *tx_hw = &dma_hw->ch[_dmaTx];
    tx_hw->read_addr  = (uint32_t)_cmdBuf;
    tx_hw->write_addr = (uint32_t)&_pio->txf[_sm];
    tx_hw->transfer_count = _cmdCount;
    tx_hw->ctrl_trig = DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS |
                       DMA_CH0_CTRL_TRIG_INCR_READ_BITS |
                       (pio_get_dreq(_pio, _sm, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);

    // ─── Configure RX DMA ─────────────────────────────────────────
    dma_channel_abort(_dmaRx);

    dma_channel_hw_t *rx_hw = &dma_hw->ch[_dmaRx];
    rx_hw->read_addr  = (uint32_t)&_pio->rxf[_sm];
    rx_hw->write_addr = (uint32_t)rxbuf;
    rx_hw->transfer_count = len;
    rx_hw->ctrl_trig = DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS |
                       DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
                       (pio_get_dreq(_pio, _sm, false) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);

    // Clear PIO interrupt
    pio_interrupt_clear(_pio, _sm);

    // ─── Switch GPIO to PIO function ──────────────────────────────
    gpio_set_function(_sda, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(_scl, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_pull_up(_sda);

    // ─── Start PIO SM, then enable DMA ────────────────────────────
    pio_sm_clear_fifos(_pio, _sm);
    pio_sm_restart(_pio, _sm);
    pio_sm_set_enabled(_pio, _sm, true);

    tx_hw->ctrl_trig |= DMA_CH0_CTRL_TRIG_EN_BITS;
    rx_hw->ctrl_trig |= DMA_CH0_CTRL_TRIG_EN_BITS;

    // ─── Wait for completion ──────────────────────────────────────
    bool ok = _waitDMADone(WIREPIO_DEFAULT_TIMEOUT_US);
    pio_sm_set_enabled(_pio, _sm, false);

    // Check for NACK
    bool nacked = pio_interrupt_get(_pio, _sm);
    pio_interrupt_clear(_pio, _sm);

    // ─── Restore GPIO to SIO mode ─────────────────────────────────
    _pioToGPIO();

    if (!ok || nacked) return 0;

    // Skip first word (address echo), extract bytes from remaining words.
    // PIO ISR with shift_in_right=false: MSB at ISR[7], LSB at ISR[0].
    // Byte is already in correct I2C order — no bit reversal needed.
    for (size_t i = 0; i < len; i++) {
        data[i] = rxbuf[i] & 0xFF;
    }
    return len;
}

size_t WirePIOTransport::pioWriteThenRead(uint8_t addr,
                                           const uint8_t *wdata, size_t wlen,
                                           uint8_t *rdata, size_t rlen) {
    if (!_pioReady || _dmaTx < 0 || _dmaRx < 0) return 0;
    if (rlen == 0) return 0;

    _buildWriteThenReadCommands(addr, wdata, wlen, rlen);

    // RX buffer: +1 for address echo from the read-address phase
    uint32_t rxbuf[WIREPIO_BUFFER_SIZE];

    // ─── Configure TX DMA ─────────────────────────────────────────
    dma_channel_abort(_dmaTx);

    dma_channel_hw_t *tx_hw = &dma_hw->ch[_dmaTx];
    tx_hw->read_addr  = (uint32_t)_cmdBuf;
    tx_hw->write_addr = (uint32_t)&_pio->txf[_sm];
    tx_hw->transfer_count = _cmdCount;
    tx_hw->ctrl_trig = DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS |
                       DMA_CH0_CTRL_TRIG_INCR_READ_BITS |
                       (pio_get_dreq(_pio, _sm, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);

    // ─── Configure RX DMA ─────────────────────────────────────────
    dma_channel_abort(_dmaRx);

    dma_channel_hw_t *rx_hw = &dma_hw->ch[_dmaRx];
    rx_hw->read_addr  = (uint32_t)&_pio->rxf[_sm];
    rx_hw->write_addr = (uint32_t)rxbuf;
    rx_hw->transfer_count = rlen;
    rx_hw->ctrl_trig = DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS |
                       DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
                       (pio_get_dreq(_pio, _sm, false) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);

    // Clear PIO interrupt
    pio_interrupt_clear(_pio, _sm);

    // ─── Switch GPIO to PIO function ──────────────────────────────
    gpio_set_function(_sda, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_set_function(_scl, _pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
    gpio_pull_up(_sda);

    // ─── Start PIO SM, then enable DMA ────────────────────────────
    pio_sm_clear_fifos(_pio, _sm);
    pio_sm_restart(_pio, _sm);
    pio_sm_set_enabled(_pio, _sm, true);

    tx_hw->ctrl_trig |= DMA_CH0_CTRL_TRIG_EN_BITS;
    rx_hw->ctrl_trig |= DMA_CH0_CTRL_TRIG_EN_BITS;

    // ─── Wait for completion ──────────────────────────────────────
    bool ok = _waitDMADone(WIREPIO_DEFAULT_TIMEOUT_US);
    pio_sm_set_enabled(_pio, _sm, false);

    // Check for NACK
    bool nacked = pio_interrupt_get(_pio, _sm);
    pio_interrupt_clear(_pio, _sm);

    // ─── Restore GPIO to SIO mode ─────────────────────────────────
    _pioToGPIO();

    if (!ok || nacked) return 0;

    // Skip first RX word (address echo)
    for (size_t i = 0; i < rlen; i++) {
        rdata[i] = rxbuf[i] & 0xFF;
    }
    return rlen;
}
