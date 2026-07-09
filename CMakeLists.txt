# WirePIO — TwoWire-compatible I2C using PIO+DMA for RP2040
# Pico SDK CMake build

cmake_minimum_required(VERSION 3.13)

# Check if we are being built as a subdirectory or standalone
if (NOT TARGET _pico_sdk_inclusion_marker)
    # Standalone: include pico_sdk_import first
    include(pico_sdk_import.cmake)

    project(WirePIO C CXX ASM)
    pico_sdk_init()
endif()

# ─── PIO Program ─────────────────────────────────────────────────────

pico_generate_pio_header(WirePIO
    ${CMAKE_CURRENT_LIST_DIR}/pio/i2c.pio
)

# ─── WirePIO Library ─────────────────────────────────────────────────

add_library(WirePIO STATIC
    src/WirePIO.cpp
    src/WirePIOTransport.cpp
    src/WirePIOSlave.cpp
)

target_include_directories(WirePIO PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(WirePIO PUBLIC
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_gpio
    hardware_clocks
    hardware_irq
    hardware_pwm
)

# ─── Examples (standalone build only) ─────────────────────────────────

if (NOT TARGET _pico_sdk_inclusion_marker)
    # I2C Scanner example
    add_executable(wirepio_scanner
        examples/pico_sdk/scanner.c
    )
    target_link_libraries(wirepio_scanner WirePIO)
    pico_add_extra_outputs(wirepio_scanner)

    # Loopback test (master-slave on two pin pairs)
    add_executable(wirepio_loopback
        examples/pico_sdk/loopback.c
    )
    target_link_libraries(wirepio_loopback WirePIO)
    pico_add_extra_outputs(wirepio_loopback)
endif()
