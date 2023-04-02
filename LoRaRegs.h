// LORAX - FSK/OOK (non-LoRa) compat library for rfm9x LoRa radios
// Copyright 2021-2022 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// based on https://github.com/sandeepmistry/arduino-LoRa
// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// non-LoRa registers
#define REG_BITRATE_MSB          0x02
#define REG_BITRATE_LSB          0x03
#define REG_FDEV_MSB             0x04
#define REG_FDEV_LSB             0x05
#define REG_PA_RAMP              0x0a
#define REG_RSSI_CONFIG          0x0e
#define REG_RSSI_VALUE           0x11
#define REG_RX_BW                0x12
#define REG_AFC_BW               0x13
#define REG_FSK_AFC_MSB          0x1b
#define REG_FSK_AFC_LSB          0x1c
#define REG_FSK_FEI_MSB          0x1d
#define REG_FSK_FEI_LSB          0x1e
#define REG_PREAMBLE_DETECT      0x1f
#define REG_FSK_PREAMBLE_MSB     0x25
#define REG_FSK_PREAMBLE_LSB     0x26
#define REG_SYNC_CONFIG          0x27
#define REG_PACKET_CONFIG_1      0x30
#define REG_PACKET_CONFIG_2      0x31
#define REG_PAYLOAD_LENGTH       0x32
#define REG_FIFO_THRESHOLD       0x35
#define REG_IMAGE_CAL            0x3b
#define REG_TEMP                 0x3c
#define REG_IRQ_FLAGS_1          0x3e
#define REG_IRQ_FLAGS_2          0x3f
#define REG_PLL_HOP              0x44
#define REG_BITRATE_FRAC         0x5d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// REG_RX_BW values
// (see table "Available RxBw Settings")
#define RXBW_2600   0b00010111
#define RXBW_3100   0b00001111
#define RXBW_3900   0b00000111
#define RXBW_5200   0b00010110
#define RXBW_6300   0b00001110
#define RXBW_7800   0b00000110
#define RXBW_10400  0b00010101
#define RXBW_12500  0b00001101
#define RXBW_15600  0b00000101
#define RXBW_20800  0b00010100
#define RXBW_25000  0b00001100
#define RXBW_31300  0b00000100
#define RXBW_41700  0b00010011
#define RXBW_50000  0b00001011
#define RXBW_62500  0b00000011
#define RXBW_83300  0b00010010
#define RXBW_100000 0b00001010
#define RXBW_125000 0b00000010
#define RXBW_166700 0b00010001
#define RXBW_200000 0b00001001
#define RXBW_250000 0b00000001

// REG_IRQ_FLAGS_1 values
#define IRQ1_MODEREADY (1<<7)
#define IRQ1_RXREADY   (1<<6)
#define IRQ1_TXREADY   (1<<5)
#define IRQ1_PLLLOCK   (1<<4)
#define IRQ1_RSSI      (1<<3)
#define IRQ1_TIMEOUT   (1<<2)
#define IRQ1_PREAMBLE  (1<<1)
#define IRQ1_SYNCMATCH (1)

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
