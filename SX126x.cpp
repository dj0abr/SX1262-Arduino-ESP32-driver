/**
 * @mainpage SX126x Arduino Library
 *
 * @section intro_sec Introduction
 *
 * Modern, clean and well-documented driver library for the Semtech SX126x LoRa transceiver.
 * 
 * - Supports LoRa and (optionally) FSK modulation
 * - Easy-to-use, modular, and robust
 * - Ready for Arduino and similar platforms
 *
 * @section basedon_sec Based On
 *
 * This library is partially based on [RadioLib](https://github.com/jgromes/RadioLib) by Jan Gromes,  
 * which provided the initial inspiration and some reference code for SX126x support.
 * 
 * All major routines have been carefully rewritten, optimized and documented for clarity and maintainability.
 *
 * @section features_sec Features
 *
 * - Fully documented code using Doxygen
 * - Configurable TX power, frequency, PA, OCP, and all major radio settings
 *
 * @section license_sec License
 * MIT License (c) 2025 DJ0ABR
 * 
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Arduino.h"
#include <SPI.h>
#include <algorithm>
#include "SX126x.h"

//#define DEBUG

/*
    Sample configurations, use the settings for your specific hardware

    // Heltec Board
    #define MOSI 10
    #define MISO 11
    #define SCK 9

    // EBYTE E220 400M22S on WROOM32
    #define MOSI 23
    #define MISO 19
    #define SCK 18
*/

/**
 * Constructor for the SX126x object.
 * Initializes the corresponding GPIO and starts the SPI interface.
 *
 * @param spiSelect   Chip Select (CS) GPIO for SPI
 * @param reset       Reset GPIO for the SX126x module
 * @param busy        Busy GPIO for the SX126x module
 * @param interrupt   Interrupt GPIO (e.g. DIO1)
 * @param SCK         SCLK GPIO for the SX126x module
 * @param MISO        MISO GPIO for the SX126x module
 * @param MOSI        MOSI GPIO for the SX126x module
 */
SX126x::SX126x(int spiSelect, int reset, int busy, int interrupt, int SCK, int MISO, int MOSI)
{
    SX126x_SPI_SELECT = spiSelect;
    SX126x_RESET      = reset;
    SX126x_BUSY       = busy;
    SX126x_INT0       = interrupt;

    // Initialize GPIOs
    pinMode(SX126x_SPI_SELECT, OUTPUT);
    pinMode(SX126x_RESET, OUTPUT);
    pinMode(SX126x_BUSY, INPUT);
    pinMode(SX126x_INT0, INPUT);

    // Initialize SPI bus (adjust pin definitions if necessary!)
    SPI.begin(SCK, MISO, MOSI, -1);

    // Debug output (optional)
    #ifdef DEBUG
    Serial.println("SX126x constructor: Pins and SPI initialized.");
    #endif
}

/**
 * Initializes the SX126x module, sets all important parameters and checks communication.
 *
 * @param packetType       Packet type (e.g. LoRa)
 * @param frequencyInHz    Operating frequency in Hz (e.g. 868000000)
 * @param txPowerInDbm     Output power in dBm (-3 to 22)
 * @param useTCXO          -1...Module uses crystal, >=0...Module's TCXO needs Dio3 voltage for VDD
 *                         use SX126X_CMD_SET_DIO3_AS_TCXO_CTRL definitions, see SX126x.h
 * @return Error code (0 = OK, <0 = Error, see SX126xError)
 */
int16_t SX126x::begin(uint8_t packetType, uint32_t frequencyInHz, int8_t txPowerInDbm, int8_t useTCXO) 
{
    // Limit to valid range
    if (txPowerInDbm > 22) txPowerInDbm = 22;
    if (txPowerInDbm < -3) txPowerInDbm = -3;
    desiredPowerDbm = txPowerInDbm;
    
    if (!Reset()) {
        #ifdef DEBUG
        Serial.println("SX126x reset failed!");
        #endif
        return SX126X_ERR_RESET_FAIL;
    }
    
    uint8_t status = GetStatus();
    if (status != 0x2A && status != 0x22) {
        #ifdef DEBUG
        Serial.printf("SX126x error status:%02X, maybe no SPI connection?",status);
        #endif
        return SX126X_ERR_STATUS_FAIL;
    }
    if (!SetStandby(SX126X_STANDBY_RC))                  return SX126X_ERR_STANDBY_FAIL;

    hasTCXO = useTCXO;
    if(hasTCXO >= 0) {
        if (!SetDio3AsTcxoCtrl(hasTCXO, RADIO_TCXO_SETUP_TIME << 6)) // TCXO Setup
            return SX126X_ERR_TCXO_FAIL;
    }

    if (!Calibrate(
        SX126X_CALIBRATE_IMAGE_ON
        | SX126X_CALIBRATE_ADC_BULK_P_ON
        | SX126X_CALIBRATE_ADC_BULK_N_ON
        | SX126X_CALIBRATE_ADC_PULSE_ON
        | SX126X_CALIBRATE_PLL_ON
        | SX126X_CALIBRATE_RC13M_ON
        | SX126X_CALIBRATE_RC64K_ON
    )) return SX126X_ERR_CALIBRATE_FAIL;

    if (!SetDio2AsRfSwitchCtrl(true))                    return SX126X_ERR_RF_SWITCH_FAIL;

    if (!SetStandby(SX126X_STANDBY_RC))                  return SX126X_ERR_STANDBY_FAIL;
    if (!SetPacketType(SX126X_PACKET_TYPE_LORA))      return -3;
    if (!SetRfFrequency(frequencyInHz))                  return SX126X_ERR_FREQ_FAIL;
    if (!configurePower(txPowerInDbm))                   return SX126X_ERR_PA_FAIL;

    if (!SetRegulatorMode(SX126X_REGULATOR_DC_DC))       return SX126X_ERR_REGULATOR_FAIL;
    if (!SetBufferBaseAddress(0, 0))                     return SX126X_ERR_BUFBASE_FAIL;
    if (!SetDioIrqParams(
        SX126X_IRQ_ALL,
        (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT),
        SX126X_IRQ_NONE,
        SX126X_IRQ_NONE
    )) return SX126X_ERR_DIO_FAIL;

    

    #ifdef DEBUG
    Serial.printf("SX126x begin() successful, Frequency: %lu Hz, TX Power: %d dBm\n", frequencyInHz, txPowerInDbm);
    #endif

    return SX126X_OK;
}

/**
 * @brief Initializes the GPIOs used for RX/TX switching.
 *
 * Stores the provided GPIO numbers internally and sets both pins as outputs.
 * Actual switching between RX and TX is handled by setRfSwitchPins().
 *
 * @param RxGPIO  GPIO number for RXEN pin (enable receiver)
 * @param TxGPIO  GPIO number for TXEN pin (enable transmitter)
 */
void SX126x::SetRxTxSwitchGPIOs(int RxGPIO, int TxGPIO) 
{
    SX126x_RXEN = RxGPIO;
    SX126x_TXEN = TxGPIO;
    pinMode(SX126x_RXEN, OUTPUT);
    pinMode(SX126x_TXEN, OUTPUT);
}

/**
 * @brief Controls the RX/TX switching using the previously configured GPIOs.
 *
 * Sets RXEN/TXEN according to the desired mode:
 * - RX mode: rxEn=true, txEn=false   → RXEN HIGH, TXEN LOW
 * - TX mode: rxEn=false, txEn=true   → RXEN LOW, TXEN HIGH
 * - Idle:    rxEn=false, txEn=false  → RXEN LOW, TXEN LOW
 *
 * @param rxEn  true to activate receiver (RX mode)
 * @param txEn  true to activate transmitter (TX mode)
 *
 * Note: If both parameters are false, both control pins will be deactivated (Idle/Sleep).
 */
void SX126x::setRfSwitchPins(bool rxEn, bool txEn) {
    if(SX126x_RXEN != -1 && SX126x_TXEN != -1) {
        if (rxEn && !txEn) {           // RX-Mode
            digitalWrite(SX126x_RXEN, HIGH);
            digitalWrite(SX126x_TXEN, LOW);
        } else if (!rxEn && txEn) {    // TX-Mode
            digitalWrite(SX126x_RXEN, LOW);
            digitalWrite(SX126x_TXEN, HIGH);
        } else {                       // Idle/Sleep/Both Off
            digitalWrite(SX126x_RXEN, LOW);
            digitalWrite(SX126x_TXEN, LOW);
        }
    }
}

/**
 * Configures the SX126x module for LoRa operation with the specified parameters.
 *
 * @param spreadingFactor LoRa spreading factor (e.g. 7 to 12)
 * @param bandwidth       LoRa bandwidth (see datasheet)
 * @param codingRate      LoRa coding rate (see datasheet)
 * @param preambleLength  Length of the preamble
 * @param crcOn           true = CRC enabled, false = no CRC
 * @param invertIQ        true = IQ inverted (typically for LoRaWAN downlink)
 * @return 0 on success, <0 on error
 */
int16_t SX126x::LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, bool crcOn, bool invertIQ) 
{
    uint8_t ldro = 0; //LowDataRateOptimize

    if (!SetStopRxTimerOnPreambleDetect(false))       return -1;
    if (!SetLoRaSymbNumTimeout(0))                    return -2;
    if (!SetPacketType(SX126X_PACKET_TYPE_LORA))      return -3;
    if (!SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro)) return -4;

    PacketParams[0] = (preambleLength >> 8) & 0xFF;
    PacketParams[1] = preambleLength;
    PacketParams[2] = 0x00;
    PacketParams[3] = 0xFF;
    PacketParams[4] = crcOn    ? 0x01 : 0x00;
    PacketParams[5] = invertIQ ? 0x01 : 0x00;

    if (!SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6, true)) return -5;

    // enable all IRQs, on DIO1: RX_DONE, TX_DONE, TIMEOUT
    if (!SetDioIrqParams(
            SX126X_IRQ_ALL,
            (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT),
            SX126X_IRQ_NONE,
            SX126X_IRQ_NONE
        )) return -6;

    // start receiving (no timeout)
    if (!SetRx(0xFFFFFF)) return -7;

    return 0;
}

/**
 * Reads a received packet from the SX126x module.
 *
 * Checks IRQ status for CRC error and RX_DONE. Reads data from the buffer if a packet was received.
 * After each call, reception is reactivated.
 *
 * @param pData Pointer to the receive buffer
 * @param len   Maximum buffer length (maximum bytes to read)
 * @return Number of received bytes (0 on error, CRC error, or nothing received)
 */
uint8_t SX126x::Receive(uint8_t *pData, uint16_t len) 
{
    // switch LoRa Module to RX
    setRfSwitchPins(true,false);

    uint8_t rxLen = 0;
    uint16_t irqRegs = GetIrqStatus();

    // First, check for CRC error
    if (irqRegs & SX126X_IRQ_CRC_ERR) {
        ClearIrqStatus(SX126X_IRQ_ALL);   // Clear all IRQs, just to be safe
        #ifdef DEBUG
        Serial.println("CRC error during reception!");
        #endif
        SetRx(0xFFFFFF);                  // Reactivate reception
        return 0;
    }

    // Then check for normal reception
    if (irqRegs & SX126X_IRQ_RX_DONE) {
        ClearIrqStatus(SX126X_IRQ_ALL);   // Also clear all IRQs here
        uint8_t result = ReadBuffer(pData, &rxLen, len);
        if (result != 0) {
            #ifdef DEBUG
            Serial.println("Error reading the buffer!");
            #endif
            SetRx(0xFFFFFF);
            return 0;
        }
        SetRx(0xFFFFFF);
        return rxLen;
    }

    // If nothing was received, still reactivate reception
    SetRx(0xFFFFFF);
    return 0;
}

/**
 * Configures all parameters for maximum TX power on SX1262 – suitable for internal and external PA modules.
 *
 * This function sets PA configuration, OCP (overcurrent protection), and output power (TX Power).
 * 
 * @param txPowerInDbm   Desired output power in dBm (-3 to 22 dBm). For external PA modules: 
 *                       The SX1262 only generates up to 22 dBm, external PA handles further amplification!
 *
 * @note
 *   - The values for SetPaConfig, SetOvercurrentProtection and SetPowerConfig refer **only to the SX1262 chip**.
 *   - The SX1262 does not know about any external PA (like with Ebyte E22-400M33S).
 *   - OCP (overcurrent protection) only limits the current through the internal PA of the SX1262 – **not** through an external PA.
 *   - For maximum output power and external PA modules, OCP should be set to a high value (e.g. 0x38 = 100mA or higher).
 *   - The actual RF power at the antenna connector depends on the module hardware, not the SX1262 register.
 *
 *   **Source: Semtech SX1262 Datasheet Chapter 5.2 & 13.1.14, Table 13-21, and Ebyte E22-400M33S Datasheet**
 *
 * @return true on success, false on error
 */
bool SX126x::configurePower(int8_t txPowerInDbm)
{
    // Limit to allowed SX1262 range (-3...22 dBm internal)
    if (txPowerInDbm > 22) txPowerInDbm = 22;
    if (txPowerInDbm < -3) txPowerInDbm = -3;

    // Default values for PA configuration (see Semtech Table 13-21)
    uint8_t paDutyCycle = 0x04;
    uint8_t hpMax       = 0x07;
    uint8_t deviceSel   = 0x00; // 0x00 = SX1262, 0x01 = SX1261 (always 0x00 for SX1262)
    uint8_t paLut       = 0x01; // 0x01 = Standard-LoRa/FSK, 0x00 = High Power (may be adjusted for ext. PA)

    // For modules with external PA like E22-400M33S, paDutyCycle/hpMax/paLut can be adjusted.
    // For typical use, the above values work with external PA too.

    // Calculate overcurrent protection (OCP): OCP = 8 + (OcpTrim × 2.5) [mA], range: 0x00...0x38
    // For maximum output (internal & external): choose high OCP (e.g. 0x38 = 100 mA)
    uint8_t ocpValue = 0x38; // Default: 100 mA
    // If you want less: uint8_t ocpValue = static_cast<uint8_t>((desiredCurrent_mA - 8) / 2.5);

    // Set settings – order is important!
    if (!SetPaConfig(paDutyCycle, hpMax, deviceSel, paLut)) return false;
    if (!SetOvercurrentProtection(ocpValue)) return false;
    if (!SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U)) return false;

    return true;
}

/**
 * Sends a data packet via the SX126x module.
 *
 * Passes data to the module, configures all required registers and waits either synchronously or asynchronously for completion.
 * 
 * @param pData Pointer to the payload to be sent
 * @param len   Payload length in bytes
 * @param mode  Transmission mode: e.g. SX126x_TXMODE_SYNC (wait for IRQ), otherwise asynchronous
 * @return true on success, false on error or timeout
 */
bool SX126x::Send(uint8_t *pData, uint8_t len, uint8_t mode)
{
    uint16_t irq;
    bool rv = false;

    // Standby & preparations
    if (!SetStandby(SX126X_STANDBY_RC)) return false;
    if (!SetDio2AsRfSwitchCtrl(true)) return false;

    if (!configurePower(desiredPowerDbm)) return false;

    PacketParams[2] = 0x00; // Variable length packet (explicit header)
    PacketParams[3] = len;

    if (!SPIwriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6, true)) {
        #ifdef DEBUG
        Serial.println("[SEND] Error: Could not set packet parameters!");
        #endif
        return false;
    }

    // Clear IRQs
    if (!ClearIrqStatus(SX126X_IRQ_ALL)) {
        #ifdef DEBUG
        Serial.println("[SEND] Error: Could not clear IRQs!");
        #endif
        return false;
    }

    // Write data to TX buffer
    if (WriteBuffer(pData, len) != 0) {
        #ifdef DEBUG
        Serial.println("[SEND] Error: WriteBuffer failed!");
        #endif
        return false;
    }

    // Activate TX (with timeout)
    if (!SetTx(5000)) {
        #ifdef DEBUG
        Serial.println("[SEND] Error: SetTx failed!");
        #endif
        return false;
    }

    if (mode & SX126x_TXMODE_SYNC)
    {
        // Synchronous wait for TX_DONE or timeout
        const uint32_t TIMEOUT = 5000; // ms
        uint32_t tStart = millis();

        do {
            irq = GetIrqStatus();
            if ((millis() - tStart) > TIMEOUT) {
                SetRx(0xFFFFFF); // Immediately reactivate reception
                #ifdef DEBUG
                Serial.println("[SEND] Timeout while waiting for TX_DONE!");
                #endif
                return false;
            }
            delay(10); // Relieves the bus
        } while ((!(irq & SX126X_IRQ_TX_DONE)) && (!(irq & SX126X_IRQ_TIMEOUT)));

        SetRx(0xFFFFFF); // Immediately activate reception

        if (irq & SX126X_IRQ_TX_DONE) {
            rv = true;
        } else {
            #ifdef DEBUG
            Serial.print("[SEND] Error: IRQ = ");
            Serial.println(irq, HEX);
            Serial.println("[SEND] Transmission aborted or timeout.");
            #endif
            rv = false;
        }
    }
    else
    {
        rv = true; // Asynchronous, assume success
    }
    return rv;
}

/**
 * Performs a hardware reset of the SX126x module.
 *
 * Sets the RESET signal to LOW for a short time, waits and checks until the module is ready again.
 *
 * @return true on success, false on timeout/error
 */
bool SX126x::Reset(void)
{
    delay(10);
    digitalWrite(SX126x_RESET, LOW);
    delay(20);
    digitalWrite(SX126x_RESET, HIGH);
    delay(10);

    // switch LoRa Module to RX
    setRfSwitchPins(false,false);

    // After reset, wait for Busy-LOW (with timeout)
    if (!waitWhileBusy(100)) {
        #ifdef DEBUG
        Serial.println("Reset: Timeout – SX126x_BUSY remains HIGH!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the SX126x module to standby mode.
 *
 * In standby mode, the chip is in low-power config mode.
 * By default after reset, STDBY_RC is active (13 MHz RC oscillator).
 *
 * @param mode 0 = STDBY_RC (13 MHz RC oscillator), 1 = STDBY_XOSC (32 MHz XTAL)
 * @return true on success, false on error
 */
bool SX126x::SetStandby(uint8_t mode)
{
    // switch LoRa Module to RX
    setRfSwitchPins(false,false);

    uint8_t data = mode;
    if (!SPIwriteCommand(SX126X_CMD_SET_STANDBY, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("SetStandby: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Reads the current status of the SX126x module.
 *
 * This command can be executed at any time and returns the current device status.
 * Also, for other SPI commands, the status is always returned as the first byte.
 *
 * Status byte:
 *   Bit 7   : unused
 *   Bit 6:4 : Chip mode (see datasheet)
 *   Bit 3:1 : Command status
 *   Bit 0   : unused
 *
 * @return Status byte (see SX126x datasheet), or 0xFF on error
 */
uint8_t SX126x::GetStatus(void)
{
    uint8_t rv = 0xFF;

    if (!SPIreadCommand(SX126X_CMD_GET_STATUS, &rv, 1, true)) {
        #ifdef DEBUG
        Serial.println("GetStatus: SPIreadCommand failed!");
        #endif
        return 0xFF;
    }
    return rv;
}

/**
 * Configures DIO3 as TCXO control and sets voltage and timeout.
 *
 * @param tcxoVoltage  Voltage for the TCXO output (only lower 3 bits used, see datasheet)
 * @param timeout      Time in units of 15.625µs, how long the signal should be present
 * @return true on success, false on error
 */
bool SX126x::SetDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout)
{
    if(hasTCXO >= 0) {
        uint8_t buf[4];
        buf[0] = tcxoVoltage & 0x07;
        buf[1] = (uint8_t)((timeout >> 16) & 0xFF);
        buf[2] = (uint8_t)((timeout >> 8) & 0xFF);
        buf[3] = (uint8_t)(timeout & 0xFF);

        if (!SPIwriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4, true)) {
            #ifdef DEBUG
            Serial.println("SetDio3AsTcxoCtrl: SPIwriteCommand failed!");
            #endif
            return false;
        }
    }
    return true;
}

/**
 * Performs the internal calibration of the SX126x.
 *
 * @param calibParam Parameter bits to select modules to be calibrated (see datasheet)
 * @return true on success, false on error
 */
bool SX126x::Calibrate(uint8_t calibParam)
{
    uint8_t data = calibParam;
    if (!SPIwriteCommand(SX126X_CMD_CALIBRATE, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("Calibrate: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Configures DIO2 as RF switch control.
 *
 * @param enable 1 = RF switch enabled, 0 = disabled
 * @return true on success, false on error
 */
bool SX126x::SetDio2AsRfSwitchCtrl(uint8_t enable)
{
    if(SX126x_RXEN == -1 && SX126x_TXEN == -1) {
        uint8_t data = enable;
        if (!SPIwriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1, true)) {
            #ifdef DEBUG
            Serial.println("SetDio2AsRfSwitchCtrl: SPIwriteCommand failed!");
            #endif
            return false;
        }
    }
    return true;
}

/**
 * Sets the operating frequency of the SX126x module (in Hz).
 *
 * Automatically performs image calibration for the selected frequency.
 *
 * @param frequency Target frequency in Hz (e.g. 868000000 for 868 MHz)
 * @return true on success, false on error
 */
bool SX126x::SetRfFrequency(uint32_t frequency)
{
    uint8_t buf[4];
    uint32_t freq = 0;

    // Perform calibration before setting frequency
    if (!CalibrateImage(frequency)) {
        #ifdef DEBUG
        Serial.println("SetRfFrequency: CalibrateImage failed!");
        #endif
        return false;
    }

    // Convert frequency to register value (FREQ_STEP)
    freq = (uint32_t)((double)frequency / (double)FREQ_STEP);

    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);

    if (!SPIwriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4, true)) {
        #ifdef DEBUG
        Serial.println("SetRfFrequency: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Performs image calibration for the SX126x module.
 *
 * Calibration values must be chosen depending on the desired operating frequency.
 *
 * @param frequency Target operating frequency in Hz (e.g. 868000000 for 868 MHz)
 * @return true on success, false on error
 *
 * @note See SX126x datasheet for calibration parameters:
 *       - 430–440 MHz:   0x6B, 0x6F
 *       - 470–510 MHz:   0x75, 0x81
 *       - 779–787 MHz:   0xC1, 0xC5
 *       - 863–870 MHz:   0xD7, 0xD8
 *       - 902–928 MHz:   0xE1, 0xE9
 */
bool SX126x::CalibrateImage(uint32_t frequency)
{
    uint8_t calFreq[2] = {0x00, 0x00};

    if (frequency > 900000000) {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    } else if (frequency > 850000000) {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    } else if (frequency > 770000000) {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    } else if (frequency > 460000000) {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    } else if (frequency > 425000000) {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    } else {
        #ifdef DEBUG
        Serial.println("CalibrateImage: Invalid frequency range!");
        #endif
        return false;
    }

    if (!SPIwriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2, true)) {
        #ifdef DEBUG
        Serial.println("CalibrateImage: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the regulator mode of the SX126x module.
 *
 * @param mode Regulator mode (e.g. 0x00 = LDO, 0x01 = DC-DC, see datasheet)
 * @return true on success, false on error
 */
bool SX126x::SetRegulatorMode(uint8_t mode)
{
    uint8_t data = mode;
    if (!SPIwriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("SetRegulatorMode: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the start addresses for the TX and RX buffers in the SX126x module.
 *
 * @param txBaseAddress Start address for the TX buffer
 * @param rxBaseAddress Start address for the RX buffer
 * @return true on success, false on error
 */
bool SX126x::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;

    if (!SPIwriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2, true)) {
        #ifdef DEBUG
        Serial.println("SetBufferBaseAddress: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the transmit power and ramp time for the SX126x module.
 *
 * @param power    Output power in dBm (-3 to +22 dBm, will be automatically limited)
 * @param rampTime Ramp time (see datasheet)
 * @return true on success, false on error
 */
bool SX126x::SetPowerConfig(int8_t power, uint8_t rampTime)
{
    uint8_t buf[2];

    // Limit to valid range
    int8_t limitedPower = std::clamp(power, (int8_t)-3, (int8_t)22);

    buf[0] = (uint8_t)limitedPower;
    buf[1] = rampTime;

    if (!SPIwriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2, true)) {
        #ifdef DEBUG
        Serial.println("SetPowerConfig: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Configures the Power Amplifier (PA) of the SX126x module.
 *
 * Note: After each call, OCP (overcurrent protection) and other parameters are set to default values!
 * For details, see the SX126X_CMD_SET_PA_CONFIG in the datasheet.
 *
 * @param paDutyCycle Duty cycle for PA (see datasheet)
 * @param hpMax       High power gain (see datasheet)
 * @param deviceSel   Selects the PA type (see datasheet)
 * @param paLut       PA lookup table (see datasheet)
 * @return true on success, false on error
 */
bool SX126x::SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;

    if (!SPIwriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4, true)) {
        #ifdef DEBUG
        Serial.println("SetPaConfig: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the overcurrent protection threshold (OCP) on the SX126x.
 *
 * The OCP can be configured in 2.5 mA steps.
 * Default value is automatically set each time SetPaConfig() is called.
 * If a different value should be used, it must be set after SetPaConfig() using SetOvercurrentProtection().
 *
 * @param value OCP value in 2.5mA steps (e.g. 0x18 = 60mA)
 * @return true on success, false on error
 */
bool SX126x::SetOvercurrentProtection(uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)((SX126X_REG_OCP_CONFIGURATION >> 8) & 0xFF); // MSB
    buf[1] = (uint8_t)(SX126X_REG_OCP_CONFIGURATION & 0xFF);        // LSB
    buf[2] = value;

    if (!SPIwriteCommand(SX126X_CMD_WRITE_REGISTER, buf, 3, true)) {
        #ifdef DEBUG
        Serial.println("SetOvercurrentProtection: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Configures the assignment of IRQ sources to DIO1, DIO2, and DIO3 pins of the SX126x.
 *
 * @param irqMask   Bitmask of enabled IRQs
 * @param dio1Mask  Bitmask of IRQs signaled on DIO1
 * @param dio2Mask  Bitmask of IRQs signaled on DIO2
 * @param dio3Mask  Bitmask of IRQs signaled on DIO3
 * @return true on success, false on error
 */
bool SX126x::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)((irqMask >> 8) & 0xFF);
    buf[1] = (uint8_t)(irqMask & 0xFF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0xFF);
    buf[3] = (uint8_t)(dio1Mask & 0xFF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0xFF);
    buf[5] = (uint8_t)(dio2Mask & 0xFF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0xFF);
    buf[7] = (uint8_t)(dio3Mask & 0xFF);

    if (!SPIwriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8, true)) {
        #ifdef DEBUG
        Serial.println("SetDioIrqParams: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Controls whether the RX timer should stop when preamble is detected.
 *
 * @param enable true: RX timer stops on preamble detection; false: timer continues
 * @return true on success, false on error
 */
bool SX126x::SetStopRxTimerOnPreambleDetect(bool enable)
{
    uint8_t data = static_cast<uint8_t>(enable);
    if (!SPIwriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("SetStopRxTimerOnPreambleDetect: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the number of LoRa® symbols for the symbol timeout in RX mode.
 *
 * In LoRa mode, the modem status may be detected as "received" too early.
 * This command sets the minimum number of received symbols before the packet is considered valid.
 *
 * @param SymbNum
 *      0:      Reception is validated as soon as one symbol is detected (default behavior)
 *      1..255: The modem logic waits until SymbNum LoRa® symbols have been received before accepting a packet as valid.
 *              Otherwise, an RxTimeout IRQ is triggered.
 * @return true on success, false on error
 */
bool SX126x::SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
    uint8_t data = SymbNum;
    if (!SPIwriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("SetLoRaSymbNumTimeout: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the packet type for the SX126x module.
 *
 * @param packetType Packet type (e.g. LoRa or FSK, see SX126x datasheet)
 * @return true on success, false on error
 */
bool SX126x::SetPacketType(uint8_t packetType)
{
    uint8_t data = packetType;
    if (!SPIwriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1, true)) {
        #ifdef DEBUG
        Serial.println("SetPacketType: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the modulation parameters of the SX126x (LoRa mode).
 *
 * @param spreadingFactor   Spreading factor (SF)
 * @param bandwidth         Bandwidth (BW)
 * @param codingRate        Coding rate (CR)
 * @param lowDataRateOptimize LDO flag (optimize for low data rate)
 * @return true on success, false on error
 */
bool SX126x::SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize)
{
    uint8_t data[4];
    // Currently only LoRa supported
    data[0] = spreadingFactor;
    data[1] = bandwidth;
    data[2] = codingRate;
    data[3] = lowDataRateOptimize;

    if (!SPIwriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4, true)) {
        #ifdef DEBUG
        Serial.println("SetModulationParams: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Reads the current IRQ status from the SX126x module.
 *
 * @return Current IRQ status (bitmask), or 0xFFFF on error
 */
uint16_t SX126x::GetIrqStatus(void)
{
    uint8_t data[2] = {0};

    if (!SPIreadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 2, true)) {
        #ifdef DEBUG
        Serial.println("GetIrqStatus: SPIreadCommand failed!");
        #endif
        return 0xFFFF;
    }
    return (data[0] << 8) | data[1];
}

/**
 * Clears the specified IRQ status in the SX126x module.
 *
 * @param irq IRQ flag(s) to be cleared (bitmask)
 * @return true on success, false on error
 */
bool SX126x::ClearIrqStatus(uint16_t irq)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)((irq >> 8) & 0xFF);
    buf[1] = (uint8_t)(irq & 0xFF);

    if (!SPIwriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2, true)) {
        #ifdef DEBUG
        Serial.println("ClearIrqStatus: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the SX126x module to receive mode (RX).
 * 
 * Timeout according to datasheet:
 *   Timeout duration [us] = timeout parameter * 15.625
 * 
 * @param timeout Timeout parameter for SX126x (already calculated)
 * @return true on success, false on error
 */
bool SX126x::SetRx(uint32_t timeout)
{
    // switch LoRa Module to RX
    setRfSwitchPins(true,false);

    uint8_t buf[3];
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);

    if (!SPIwriteCommand(SX126X_CMD_SET_RX, buf, 3, true)) {
        #ifdef DEBUG
        Serial.println("SetRx: SPIwriteCommand failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sets the SX126x module to transmit mode (TX mode).
 * 
 * After sending the last bit, the IRQ TX_DONE is triggered.
 * If this does not happen within the timeout, a TIMEOUT IRQ is triggered.
 * After TX_DONE or TIMEOUT, the module returns to STBY_RC mode.
 * 
 * Timeout calculation according to datasheet:
 *  Timeout duration [us] = timeout parameter * 15.625
 * 
 * @param timeoutInMs Timeout in milliseconds (0 = no timeout, TX mode until packet sent)
 * @return true on success, false on error/timeout
 */
bool SX126x::SetTx(uint32_t timeoutInMs)
{
    SetDio2AsRfSwitchCtrl(true);
    // switch LoRa Module to TX
    
    // Switch to standby RC
    if (!SetStandby(SX126X_STANDBY_RC)) {
        #ifdef DEBUG
        Serial.println("SetTx: Error switching to standby!");
        #endif
        return false;
    }
    setRfSwitchPins(false,true);

    uint8_t buf[3];
    uint32_t tout = timeoutInMs;
    if (timeoutInMs != 0) {
        // Convert timeout to "SX126x ticks": Timeout [ticks] = Timeout_in_us / 15.625
        uint32_t timeoutInUs = timeoutInMs * 1000;
        tout = (uint32_t)(timeoutInUs / 15.625);
    }

    buf[0] = (uint8_t)((tout >> 16) & 0xFF);
    buf[1] = (uint8_t)((tout >> 8) & 0xFF);
    buf[2] = (uint8_t)(tout & 0xFF);

    // Send command
    if (!SPIwriteCommand(SX126X_CMD_SET_TX, buf, 3, true)) {
        #ifdef DEBUG
        Serial.println("SetTx: SPIwriteCommand failed!");
        #endif
        return false;
    }

    // Wait for TX_WAIT status (max. 10ms)
    for (int retry = 0; retry < 10; retry++) {
        if ((GetStatus() & 0x70) == 0x60) return true; // TX_WAIT
        delay(1);
    }

    // Final status check, debug output on error
    uint8_t s = GetStatus();
    if ((s & 0x70) != 0x60) {
        #ifdef DEBUG
        Serial.print("SetTx: Illegal status (should be TX_WAIT, 0x60) but is: 0x");
        Serial.println(s, HEX);
        #endif
        return false;
    }
    return true;
}

/**
 * Reads the status of the RX buffer.
 *
 * @param payloadLength           Pointer, return: length of the payload in the RX buffer
 * @param rxStartBufferPointer    Pointer, return: start position in the RX buffer
 * @return true on success, false on error
 */
bool SX126x::GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
    uint8_t buf[2];

    // Send command to the SX126x module and receive 2 bytes
    if (!SPIreadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2, true)) {
        #ifdef DEBUG
        Serial.println("GetRxBufferStatus: Error reading RX buffer status!");
        #endif
        *payloadLength = 0;
        *rxStartBufferPointer = 0;
        return false;
    }

    *payloadLength = buf[0];
    *rxStartBufferPointer = buf[1];
    return true;
}

/**
 * Reads the RX buffer of the SX126x.
 *
 * @param rxData    Pointer to buffer for received data
 * @param rxDataLen Pointer to variable where the length of received data will be stored (overwritten)
 * @param maxLen    Maximum buffer size of rxData
 * @return 0 on success, 1 if too much data, 2 on timeout/error
 */
uint8_t SX126x::ReadBuffer(uint8_t *rxData, uint8_t *rxDataLen, uint8_t maxLen)
{
    uint8_t offset = 0;

    // Get status & offset of RX buffer
    GetRxBufferStatus(rxDataLen, &offset);

    // Check buffer overflow
    if (*rxDataLen > maxLen) {
        #ifdef DEBUG
        Serial.println("ReadBuffer: Received data too long for the buffer!");
        #endif
        return 1;
    }

    // Wait for Busy before transfer
    if (!waitWhileBusy(50)) {
        #ifdef DEBUG
        Serial.println("ReadBuffer: Timeout before SPI transfer.");
        #endif
        return 2;
    }

    // Start SPI transfer
    digitalWrite(SX126x_SPI_SELECT, LOW);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    // Command sequence according to SX126x datasheet
    SPI.transfer(SX126X_CMD_READ_BUFFER);
    SPI.transfer(offset);            // Start position in RX buffer
    SPI.transfer(SX126X_CMD_NOP);    // Dummy byte (status)

    // Read data
    for (uint16_t i = 0; i < *rxDataLen; i++) {
        rxData[i] = SPI.transfer(SX126X_CMD_NOP);
    }

    // End SPI transfer
    SPI.endTransaction();
    digitalWrite(SX126x_SPI_SELECT, HIGH);

    // Wait for Busy after transfer
    if (!waitWhileBusy(50)) {
        #ifdef DEBUG
        Serial.println("ReadBuffer: Timeout after SPI transfer.");
        #endif
        return 2;
    }

    return 0;
}

/**
 * Writes data to the TX buffer of the SX126x.
 * 
 * @param txData     Pointer to data to be sent
 * @param txDataLen  Length of data to be sent
 * @return 0 on success, 1 on timeout/error
 */
uint8_t SX126x::WriteBuffer(uint8_t *txData, uint8_t txDataLen)
{
    // Wait for Busy before transfer
    if (!waitWhileBusy(50)) {
        #ifdef DEBUG
        Serial.println("WriteBuffer: Timeout before SPI transfer.");
        #endif
        return 1;
    }

    // Start SPI transfer
    digitalWrite(SX126x_SPI_SELECT, LOW);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    // Command byte and FIFO offset (always 0)
    SPI.transfer(SX126X_CMD_WRITE_BUFFER);
    SPI.transfer(0); 

    // Transfer payload
    for (uint16_t i = 0; i < txDataLen; i++) {
        SPI.transfer(txData[i]);
    }

    // End SPI transfer
    SPI.endTransaction();
    digitalWrite(SX126x_SPI_SELECT, HIGH);

    // Wait for Busy after transfer
    if (!waitWhileBusy(50)) {
        #ifdef DEBUG
        Serial.println("WriteBuffer: Timeout after SPI transfer.");
        #endif
        return 1;
    }

    return 0;
}

/**
 * Sends an SPI write command to the SX126x module.
 *
 * @param cmd         Command byte
 * @param data        Pointer to data to be sent
 * @param numBytes    Number of bytes to send
 * @param waitForBusy true, if wait for BUSY after transfer
 */
bool SX126x::SPIwriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    if (!SPItransfer(cmd, true, data, NULL, numBytes, waitForBusy)) {
        #ifdef DEBUG
        Serial.println("SPIwriteCommand: SPI transfer failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Sends an SPI read command to the SX126x module and reads data.
 *
 * @param cmd         Command byte
 * @param data        Pointer to buffer for read data
 * @param numBytes    Number of bytes to read
 * @param waitForBusy true, if wait for BUSY after transfer
 */
bool SX126x::SPIreadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
    if (!SPItransfer(cmd, false, NULL, data, numBytes, waitForBusy)) {
        #ifdef DEBUG
        Serial.println("SPIreadCommand: SPI transfer failed!");
        #endif
        return false;
    }
    return true;
}

/**
 * Performs an SPI transfer with the SX126x module.
 * 
 * @param cmd         Command code for SX126x
 * @param write       true = write operation, false = read operation
 * @param dataOut     Pointer to data to be sent (write only)
 * @param dataIn      Pointer to buffer for read data (read only)
 * @param numBytes    Number of bytes to transfer
 * @param waitForBusy true: Wait for BUSY after transfer
 */
bool SX126x::SPItransfer(uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy) {
    // Wait for BUSY LOW (before transfer)
    if (!waitWhileBusy(50)) {
        #ifdef DEBUG
        Serial.println("SPIreadCommand: Aborted because BUSY too long HIGH.");
        #endif
        return false;
    }

    // Start SPI transfer
    digitalWrite(SX126x_SPI_SELECT, LOW);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    // Send command
    SPI.transfer(cmd);

    // Transfer data
    if (write) {
        for (uint8_t n = 0; n < numBytes; n++) {
            SPI.transfer(dataOut[n]);
        }
    } else {
        // On read: first dummy byte (status)
        SPI.transfer(SX126X_CMD_NOP);
        for (uint8_t n = 0; n < numBytes; n++) {
            dataIn[n] = SPI.transfer(SX126X_CMD_NOP);
        }
    }

    // End SPI transfer
    SPI.endTransaction();
    digitalWrite(SX126x_SPI_SELECT, HIGH);

    // Optional: Wait for BUSY LOW after transfer
    if (waitForBusy) {
        delayMicroseconds(1);
        // Wait for BUSY LOW (after transfer)
        if (!waitWhileBusy(50)) {
            #ifdef DEBUG
            Serial.println("SPIreadCommand: Aborted because BUSY too long HIGH.");
            #endif
            return false;
        }
    }
    return true;
}

/**
 * Waits until the SX126x_BUSY pin is LOW or a timeout occurs.
 * 
 * @param timeout_ms Timeout in milliseconds
 * @return true if BUSY went LOW; false on timeout
 */
bool SX126x::waitWhileBusy(uint32_t timeout_ms) {
    unsigned long start = millis();
    while (digitalRead(SX126x_BUSY)) {
        if (millis() - start > timeout_ms) {
            #ifdef DEBUG
            Serial.println("Error: SX126x_BUSY timeout.");
            #endif
            return false;
        }
    }
    return true;
}
