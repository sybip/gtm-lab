
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include <TimeLib.h>
#include <SPI.h>

#include "LoRaX.h"
#include "LoRaRegs.h"
#include "RS-mod-FCR.h"

//
// USER CONFIGURABLE OPTIONS
// -------------------------
// These are make-or-break settings, so choose carefully
//
// ISM_REGION: 1=US/CA, 2=EU, 4=AU/NZ, 6=TW/JP
#define ISM_REGION 1
// BOARD_TYPE: 1=TBEAM-1.x, 2=WRL-15006
#define BOARD_TYPE 1
// Logging verbosity, please use sparingly:
// - DEBUG may distort protocol timing causing some message loss
// - VERBOSE **will** break the protocol timings and prevent
//       any message reception beyond individual packets
#define VERBOSITY ESP_LOG_INFO
// END of user options


// Logging functions and options
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  // DO NOT EDIT THIS LINE
#define TAG "GTMLAB"
#include "esp_log.h"

#define LOGE( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   TAG, format, ##__VA_ARGS__)
#define LOGW( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    TAG, format, ##__VA_ARGS__)
#define LOGI( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    TAG, format, ##__VA_ARGS__)
#define LOGD( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   TAG, format, ##__VA_ARGS__)
#define LOGV( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, format, ##__VA_ARGS__)

// HARDWARE SETTINGS
#if BOARD_TYPE==1
#define pinCS 18   // LoRa radio chip select
#define pinRST 23  // LoRa radio reset
#define pinIRQ 26  // Hardware interrupt pin
#define PIN_SCLK 5
#define PIN_MISO 19
#define PIN_MOSI 27

#elif BOARD_TYPE==2
// Settings for Sparkfun WRL-15006 (ESP32 Lora Gateway)
#define pinCS 16  // LoRa radio chip select
#define pinRST 27  // LoRa radio reset
#define pinIRQ 26  // Hardware interrupt pin

#else
#error "Invalid BOARD_TYPE"
#endif

// FREQUENCY BAND SETTINGS
#if ISM_REGION==1
// REGION=US
#define BASEFREQ 902500000
#define CHANSTEP 500000
#define DATACHANS { 7, 42, 41, 46, 48, 45, 11, 18, \
                    6, 2, 26, 13, 19, 39, 37, 43, \
                    10, 15, 20, 50, 16, 21, 32, 22, \
                    23, 0, 40, 9, 35, 33, 28, 34, \
                    24, 31, 17, 30, 27, 44, 14, 4, \
                    3, 29, 38, 36, 5, 12, 8, 47 }
#define CTRLCHANS { 1, 25, 49 }

#elif ISM_REGION==2
// REGION=EU
#define BASEFREQ 869425000
#define CHANSTEP 50000
#define DATACHANS { 1, 2, 3 }
#define CTRLCHANS { 0,4 }

#elif ISM_REGION==4
// REGION=ANZ
#define BASEFREQ 915500000
#define CHANSTEP 500000
#define DATACHANS { 7, 11, 18, 6, 2, 13, 19, 10, \
                    15, 20, 16, 21, 22, 0, 9, 24, \
                    17, 14, 4, 3, 5, 8 }
#define CTRLCHANS { 1, 12, 23 }

#elif ISM_REGION==6
// REGION=JP
#define BASEFREQ 920500000
#define CHANSTEP 500000
#define DATACHANS { 7, 6, 2, 1, 4, 3, 5 }
#define CTRLCHANS { 0, 8 }

#else
#error "Invalid ISM_REGION"
#endif

uint8_t dataChans[] = DATACHANS;
uint8_t ctrlChans[] = CTRLCHANS;

// Common settings for GTM waveform
int bitrate = 24000;
int freqdev = 12500;

#define RX_TIMEOUT 400 // millis
#define SCAN_DWELL 10  // millis
#define FXOSC 32E6

// Data buffers
uint8_t radioBuf[256];    // radio packet buffer
uint8_t radioDec[248];    // reedsolo decode buffer
uint8_t packetBuf[512];   // "a CVE waiting to happen"
int packetLen=0;
int radioLen=0;
int wantLen=0;

// Useful values from last sync packet
uint8_t wantFrags=0; // Number of fragments we expect
uint8_t lastIniTTL;  // for standardized output
uint8_t lastCurTTL;  // for standardized output

// Channel hopping related variables
bool scanning = true;
bool recvData = false;  // if true, we are on a data chan
uint8_t currChan = 0;   // current channel NUMBER
uint8_t currDChIdx = 0; // current data chan INDEX in map
uint8_t currCChIdx = 0; // current ctrl chan INDEX in map

// Timing related variables
unsigned long dataStart=0;  // millis when data RX started
unsigned long lastHop=0;    // millis when we last hopped channel

// Interrupt Status Registers - stored previous values
uint8_t prev_ISR1=0;
uint8_t prev_ISR2=0;


// additive CRC16 function
uint16_t CRC16_add(uint8_t b, uint16_t crc = 0) {
  for (int j = 0x80; j > 0; j >>= 1) {
    uint16_t bit = (uint16_t)(crc & 0x8000);
    crc <<= 1;

    if ((b & j) != 0)
      bit = (uint16_t)(bit ^ 0x8000);

    if (bit != 0)
      crc ^= 0x1021;  // polynomial
  }
  return crc;
}


// GTH16 hash algorithm (based on park-miller LCG)
// WARNING: 64-bit multiplication used
uint16_t gtAlgoH16(uint8_t* str, size_t len) 
{
    uint32_t seed = 0xaa;
    uint32_t mult = 48271;
    uint32_t incr = 1;
    uint32_t modulus = (1<<31) -1; // 0x7FFFFFFF;

    uint32_t h = 0;
    uint64_t x = seed;
    for (int i=0; i<len; i++) {
        x = (((x + str[i]) * mult + incr) & 0xFFFFFFFF) % modulus;
        h ^= x;
    }

    // Derive 16-bit value from 32-bit hash by XORing its two halves
    return ((h & 0xFFFF0000) >> 16) ^ (h & 0xFFFF);
}


// Set frequency, argument is channel number (0 to NUMCHANS)
void setChan(uint8_t chan)
{
  if (!scanning) {
    // don't pollute the log with control channel scanning hops
    LOGD("SetChan: %d->%d", currChan, chan);
  }
  LoRa.setFrequency(BASEFREQ + (chan*CHANSTEP));
  currChan = chan;
  bool locked = false;
  while(!locked) {
    uint8_t curr_ISR1 = LoRa.readRegister(REG_IRQ_FLAGS_1);
    if ((curr_ISR1 & 0xd0) == 0xd0)
      locked = true;
  }
}

// Tune to a data channel, argument is INDEX in dataChans[] map!
//   (do not use a channel number as argument)
// To hop to a channel by number, use instead setChan() above
void setDataChanX(uint8_t dChanX)
{
  dChanX %= sizeof(dataChans);
  setChan(dataChans[dChanX]);
}

// Tune to a control channel (next in sequence and increment)
// Call this repeatedly to traverse channel map in a scanning pattern
void setCtrlChan()
{
  currCChIdx++;
  currCChIdx %= sizeof(ctrlChans);
  setChan(ctrlChans[currCChIdx]);
}


// Reset receiver soft buffers and state variables
// Call this after successful reception of a data message, or
//   to bail out of a receive operation that failed partway
void resetState()
{
  LOGD("----------------------------------------");
  recvData = false;
  wantFrags = 0;
  radioLen = 0;
  packetLen = 0;
  memset(radioBuf, 0, sizeof(radioBuf));
  memset(packetBuf, 0, sizeof(packetBuf));
  // expect long preamble
  LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xca); // 3 bytes
  setCtrlChan();
  scanning = true;
}


void setup()
{
  int bitdivisor = int(FXOSC/bitrate);
  
  Serial.begin(115200);     // initialize serial console
  while (!Serial);          // recommended if USB serial

  esp_log_level_set("*", VERBOSITY);

#ifdef PIN_SCLK
  // radio sits on a non-default SPI
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
#endif

  LOGI("Receiver startup: board=%d, region=%d", BOARD_TYPE, ISM_REGION);

  // Configure the CS, reset, and IRQ pins
  LoRa.setPins(pinCS, pinRST, pinIRQ);

  if (!LoRa.begin(BASEFREQ)) {   // init radio at base frequency
    LOGE("Radio init failed. Check pin definitions");
    while (true);
  }

  // Check radio version
  if (LoRa.readRegister(REG_VERSION) != 0x12) {
    LOGE("Unknown radio device");
    while (true);
  }

  // Set operating mode - sleep
  LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP);
  LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP); // the LoRa bit can only be written in sleep mode

  // Set bitrate
  LoRa.writeRegister(REG_BITRATE_MSB, (uint8_t) (bitdivisor >> 8));
  LoRa.writeRegister(REG_BITRATE_LSB, (uint8_t) (bitdivisor & 0xFF));

  // Set frequency deviation FIXME show working out
  LoRa.writeRegister(REG_FDEV_MSB, (uint8_t) 0);
  LoRa.writeRegister(REG_FDEV_LSB, (uint8_t) 0xcd);

  // Set PA RAMP modulation shaping
  LoRa.writeRegister(REG_PA_RAMP, (uint8_t) 0x49);  // Gaussian BT=0.5
  
  // Set filter bandwidth
  LoRa.writeRegister(REG_RX_BW, 0x0c);  // 25.0 kHz (see table 38)

  // Set preamble detection
  LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xca); // 3 bytes preamble
  //LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xaa); // 2 bytes preamble
  //LoRa.writeRegister(REG_PREAMBLE_DETECT, 0x8a); // 1 bytes preamble

  // Set Sync configuration
  LoRa.writeRegister(REG_SYNC_CONFIG, 0x51);  // not 81

  // Set Sync Word 0x2d 0xd4
  LoRa.writeRegister(REG_SYNC_CONFIG+1, 0x2d);
  LoRa.writeRegister(REG_SYNC_CONFIG+2, 0xd4);

  // Set packet format - variable length
  LoRa.writeRegister(REG_PACKET_CONFIG_1, 0x80);

  // Set maximum payload length - 255 should be enough
  LoRa.writeRegister(REG_PAYLOAD_LENGTH, 0xFF);

  // RegPLL Hop - set fast hopping
  LoRa.writeRegister(REG_PLL_HOP, (0x2d | 0x80));

  // Start RX mode
  LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

  // Set frequency
  setCtrlChan();

  // Dump registers
  for (int i = 0; i < 0x60; i++) {
    LOGD("0x%02x: 0x%02x", i, LoRa.readRegister(i));
  }
  radioLen = 0;
  LOGI("Receiver up and running");
}


void loop()
{
  String apiOut = "";
  char rx_byte = 0;
  uint8_t curr_ISR1=0;
  uint8_t curr_ISR2=0;
  bool payReady = false;

  // Read ISR registers once
  curr_ISR2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
  curr_ISR1 = LoRa.readRegister(REG_IRQ_FLAGS_1);

  if (curr_ISR1 & 0x02)  // Preamble detected
      scanning = false;

  if ((curr_ISR1 != prev_ISR1) || (curr_ISR2 != prev_ISR2)) {
    LOGV("IRQs: %02x|%02x", curr_ISR1, curr_ISR2);
    if (((prev_ISR1 & 0x02)!=0x02) && ((curr_ISR1 & 0x02)==0x02)) {
      LOGD("PREAMBLE");
    }

    if (((prev_ISR1 & 0x03)!=0x03) && ((curr_ISR1 & 0x03)==0x03)) {
      LOGD("SYNCWORD");
      radioLen = 0;
      memset(radioBuf, 0, 256);
      scanning = false;
    }

    if (((prev_ISR2 & 0x04)!=0x04) && ((curr_ISR2 & 0x04)==0x04)) {
      LOGD("PAYREADY");
    } 

    if ((curr_ISR2 & 0x04)==0x04) {
      payReady = true;
    }

    if (((prev_ISR1 & 0x03)==0x03) && ((curr_ISR1 & 0x03)!=0x03)) {
      if ((radioLen > 0) && !payReady) {
        // lost sync while receiving packet - NOT GOOD
        LOGI("LOSTSYNC at pos %d/%d", radioLen-1, radioBuf[0]);
        resetState();
      }
    }

    // store values as "previous"
    prev_ISR1=curr_ISR1;
    prev_ISR2=curr_ISR2;
  }

  if(((curr_ISR1 & 0x03)==0x03) && 
     ((curr_ISR2 & 0x20) || !(curr_ISR2 & 0x40))) {
    radioBuf[radioLen] = LoRa.readRegister(REG_FIFO);
    // LOGV("< %02x", radioBuf[radioLen]);
    radioLen++;

    // if ((radioLen>0) && (radioLen>radioBuf[0])) {  // first byte is len
    // use PAYLOAD_READY ISR bit instead
    if (payReady) {
      LOGD("rxLen=%d", radioLen);
      ESP_LOG_BUFFER_HEXDUMP(TAG, radioBuf, radioLen, ESP_LOG_VERBOSE);
      if (radioLen>10) {
        RS::ReedSolomon rs;
        rs.begin(radioLen-8, 1);
        if (rs.Decode(radioBuf, radioDec) == 0) {
          LOGD("REEDSOLO");
          // Packet OK, send it for further processing
          rxPacket(radioLen-8);
        }
      }
      radioLen = 0;
      memset(radioBuf, 0, 256);
    }
  }

  if (recvData && (millis()-dataStart > RX_TIMEOUT)) {
    LOGI("RX_STALL");
    resetState();
  }

  if (scanning && !recvData) {
    // if scanning, hop to next chan every SCAN_DWELL millis
    if ((millis()-lastHop) > SCAN_DWELL) {
      setCtrlChan();
      lastHop = millis();
    }
  }
}


// called from radio receiver for each good packet received
int rxPacket(uint8_t size)
{
  ESP_LOG_BUFFER_HEXDUMP(TAG, radioDec, size, ESP_LOG_VERBOSE);

  uint16_t msgH16;

  uint16_t crc_want = (radioDec[size-2]<<8) + radioDec[size-1];
  uint16_t crc_calc=0;
  for (int i=0; i<size-2; i++) {
    crc_calc=CRC16_add(radioDec[i], crc_calc);
  }
  crc_calc ^= 0xabcd;
  
  if (crc_want == crc_calc) {
    LOGD("CRC16-OK");
  } else {
    LOGI("CRC16BAD want:%04x, got:%04x", crc_want, crc_calc);
    return 0;
  }

  // good packet; drop the CRC
  size-=2;

  // formatted channel information for debug output
  char chanDesc[16];
  if (recvData) {
    sprintf(chanDesc, "RX DCh=%02d", currChan);
  } else {
    sprintf(chanDesc, "RX CCh=%02d", currChan);
  }

  // First byte is raw packet length (ignorable by now)
  // Second byte is packet type
  if (radioDec[1]==1) {
    // SYNC packet, indicates the begining of a data packet
    LOGI("%s SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
      chanDesc, radioDec[2], radioDec[3], radioDec[4], radioDec[5]);
      currDChIdx = radioDec[2];
      wantFrags = radioDec[3];
      // Save a copy
      lastIniTTL = radioDec[4];
      lastCurTTL = radioDec[5];
      LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xaa); // 2 bytes
      setDataChanX(++currDChIdx);
      dataStart = millis();
      recvData = true;

  } else if (radioDec[1]==3) {
    // ACK packet, indicates the successful delivery of a P2P message
    LOGI("%s ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d",
      chanDesc, (radioDec[2]<<8)+radioDec[3], 
      (radioDec[4]>>4), (radioDec[4] & 0x0f), radioDec[5]);

    // output in standardized format
    printf("RX_ACK:");
    for (int i=2; i<size; i++)
      printf("%02x", radioDec[i]);
    printf("\n");
    resetState();

  } else if (radioDec[1]==2) {
    // DATA packet, a fragment of a protocol message of random length
    LOGI("%s DATA(2): len=%d, fragIDX=%d", chanDesc, radioDec[2], radioDec[3]);
    memcpy((packetBuf+packetLen), (radioDec+4), radioDec[2]);
    packetLen += radioDec[2];
    wantFrags--;

    if (wantFrags) {
      // hop to next data channel to receive more fragments
      setDataChanX(++currDChIdx);
    } else {
      // no more frags; dispatch packet and hop to control channel
      // ...but first, calculate GTH16 hash
      if ((packetBuf[0]==2) || (packetBuf[0]==2)) {
        msgH16 = gtAlgoH16(packetBuf+5, 16);   // location of HEAD element
      } else {
        msgH16 = gtAlgoH16(packetBuf+15, 16);  // location of HEAD element
      }
      LOGI("complete: len=%d, hash=0x%04x, time=%dms", packetLen, msgH16, (millis()-dataStart));

      // output in standardized format (inittl, curttl, | ,hexmsg)
      printf("RX_MSG:");
      printf("%02x%02x|", lastIniTTL, lastCurTTL);
      for (int i=0; i<packetLen; i++)
        printf("%02x", packetBuf[i]);
      printf("\n");
      packetLen=0;
      // will reset buffers, hop to cch and start scanning
      resetState();
    }

  } else {
    LOGI("%s UNK (%d)", chanDesc, radioDec[1]);    
  }
}
