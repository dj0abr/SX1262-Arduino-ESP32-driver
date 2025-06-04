# SX126x Arduino/ESP32 Library

A modern, clean, and well-documented Arduino library for the Semtech SX126x LoRa transceivers (SX1262, SX1268, LLCC68).

**Designed by [DJ0ABR](https://github.com/DJ0ABR)**

[![MIT License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Arduino-blue)](https://www.arduino.cc/)

---

## Features

- **Full control** over LoRa configuration, transmit power, OCP, frequency, and more.
- **TCXO support:** Ideal for modules such as E22-400M33S, Heltec WiFi LoRa V3, and others.
- **Well documented:** Every feature is commented using Doxygen.
- **Arduino compatible:** Works on ESP32, AVR, STM32, RP2040, and more.
- **Modular and clean:** Easy to use in your own projects.

---

## Installation

1. Copy the library into your Arduino project or `libraries` directory.
2. Add `#include "SX126x.h"` to your sketch.
3. Adjust SPI and (optionally) RXEN/TXEN pins to match your hardware.

---

## Wiring Example

For Heltec WiFi LoRa 32 V3 board:

| SX126x Pin | Board Pin | Arduino Macro         |
|------------|-----------|----------------------|
| MOSI       | 10        | `#define MOSI 10`    |
| MISO       | 11        | `#define MISO 11`    |
| SCK        | 9         | `#define SCK 9`      |
| NSS        | ...       | e.g. D8              |
| RESET      | ...       | e.g. D12             |
| BUSY       | ...       | e.g. D13             |
| DIO1       | ...       | e.g. D14             |

More wiring examples can be found in the source file and below in the minimal example.

---

## Minimal Example

Minimal code to configure the SX126x/LLCC68 LoRa module. When configuration is complete simply use Send(...) or Receive(...) to transmit and receive data.

```cpp
#define RF_FREQUENCY         433000000
#define TX_OUTPUT_POWER      22
#define LORA_BANDWIDTH       4
#define LORA_SPREADING_FACTOR 9
#define LORA_CODINGRATE      4
#define LORA_PREAMBLE_LENGTH 8

// Example board: Heltec Wifi Lora V3 Board
#define MOSI 10
#define MISO 11
#define SCK 9

LORA::LORA()
    : lora(LORA_NSS, LORA_RST, LORA_BUSY, LORA_DIO1, SCK, MISO, MOSI)
{
}

bool LORA::begin() {
    int err = lora.begin(
        SX126X_PACKET_TYPE_LORA,   // LoRa mode
        RF_FREQUENCY,              // frequency in Hz
        TX_OUTPUT_POWER,           // tx power in dBm
        SX126X_DIO3_OUTPUT_1_8     // TCXO VDD voltage via Dio3
    );
    if (err != 0) {
        printf("lora.begin error: %d\n", err);
        return false;
    }
    lora.LoRaConfig(
        LORA_SPREADING_FACTOR,
        LORA_BANDWIDTH,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        true,   // CRC on
        false   // invertIQ
    );
    return true;
}
```

---

## Notes & Tips

- **TCXO:** For boards with an internal TCXO (e.g., Heltec V3, E22-400M33S), set `useTCXO` to `SX126X_DIO3_OUTPUT_1_8`.
- **External PA:** For modules with an external PA (such as E22-400M33S), ensure you provide enough supply voltage (typically ≥5 V for full 2 W output).
- **Ext.PA control:** use SetRxTxSwitchGPIOs(int RxGPIO, int TxGPIO) if the module uses external RX/TX pins, like EBYTE
- **Compatibility:** The library is designed for all SX126x chips. Not all features are available on every chip or module.

---

## Contributing & Feedback

Pull requests and issues are very welcome!
Please report bugs, feature requests, or suggestions directly via GitHub.

---

## Credits

- **Based on [RadioLib](https://github.com/jgromes/RadioLib) by Jan Gromes** – thanks for the inspiration and reference code!
- **Copyright:** (c) 2025 DJ0ABR  
- **License:** MIT

---

Enjoy & 73!
