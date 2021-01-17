//
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
#define CHANNELS 51
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
#define CHANNELS 5
#define DATACHANS { 1, 2, 3 }
#define CTRLCHANS { 0, 4 }

#elif ISM_REGION==4
// REGION=ANZ
#define BASEFREQ 915500000
#define CHANSTEP 500000
#define CHANNELS 25
#define DATACHANS { 7, 11, 18, 6, 2, 13, 19, 10, \
                    15, 20, 16, 21, 22, 0, 9, 24, \
                    17, 14, 4, 3, 5, 8 }
#define CTRLCHANS { 1, 12, 23 }

#elif ISM_REGION==6
// REGION=JP
#define BASEFREQ 920500000
#define CHANSTEP 500000
#define CHANNELS 9
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

// Some constants related to radio timings etc,
// Determined mostly through trial-and-error
#define RX_TIMEOUT 400 // millis
#define SCAN_DWELL 10  // millis
uint8_t txSyncDelay = 10;  // millis to wait between sync packet and first data packet
uint8_t txPackDelay = 8;   // millis to wait between data packets
uint8_t TXFRAGSIZE = 90;   // it's what they use
// min and max acceptable RX packet sizes (including RS ECC)
#define RX_MIN_LEN 10   // min
#define RX_MAX_LEN 200  // max


#define FXOSC 32E6  // 32MHz

// Radio packet types
#define PKT_TYPE_SYNC 1
#define PKT_TYPE_DATA 2
#define PKT_TYPE_ACK  3

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
bool inTXmode = false;
bool recvData = false;  // if true, we are on a data chan
uint8_t currChan = 0;   // current channel NUMBER
uint8_t currDChIdx = 0; // current data chan INDEX in map
uint8_t currCChIdx = 0; // current ctrl chan INDEX in map

// Timing related variables
unsigned long pktStart=0;   // millis when packet RX started
unsigned long dataStart=0;  // millis when data RX started
unsigned long lastHop=0;    // millis when we last hopped channel

// Interrupt Status Registers - stored previous values
uint8_t prev_ISR1=0;
uint8_t prev_ISR2=0;

// Serial console buffer
char conBuf[256];
uint16_t conLen=0;


// Binary output macros
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 


// additive CRC16 function
uint16_t CRC16_add(uint8_t b, uint16_t crc = 0)
{
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


// Dump all radio registers, in hex and binary (datasheet for details)
// Argument: ESP log level, -1 for printf
void dumpRegisters(int logLevel)
{
  for (int i = 0; i < 0x70; i++) {
    uint8_t rVal = LoRa.readRegister(i);
    if (logLevel<0) {
      printf("0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN"\n", i, rVal, BYTE_TO_BINARY(rVal));
    } else {
      ESP_LOG_LEVEL_LOCAL(logLevel, TAG, 
        "0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN, i, rVal, BYTE_TO_BINARY(rVal));      
    }
  }
}


// Set frequency, argument is channel number (0 to NUMCHANS)
void setChan(uint8_t chan)
{
  unsigned long tStart = millis();

  LoRa.setFrequency(BASEFREQ + (chan*CHANSTEP));

  bool locked = false;
  while(!locked) {
    uint8_t curr_ISR1 = LoRa.readRegister(REG_IRQ_FLAGS_1);
    // MODEREADY | PLLLOCK = 0b10010000
    if ((curr_ISR1 & 0x90) == 0x90)
      locked = true;
  }

  if (inTXmode || (!scanning)) {
    // don't pollute the log with control channel scanning hops
    LOGD("SetChan: %d->%d (%dms)", currChan, chan, (millis() - tStart));
  }

  currChan = chan;
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
// Call this after a successful object reception or transmission, 
//   or to bail out of a receive operation that failed partway
void resetState()
{
  LOGD("----------------------------------------");
  wantFrags = 0;
  radioLen = 0;
  packetLen = 0;
  memset(radioBuf, 0, sizeof(radioBuf));
  memset(packetBuf, 0, sizeof(packetBuf));

  // expect long preamble
  LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xca); // 3 bytes

  // switch to RX mode in case we're not
  LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

  scanning = true;    // cycling through ctrl chans 
  inTXmode = false;   // we're not in a transmission
  recvData = false;   // we're not receiving data
  setCtrlChan();
}


// Call from Arduino setup() to initialize radio and data structures
void gtmlabSetup()
{
#ifdef PIN_SCLK
  // radio sits on a non-default SPI
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
#endif

  uint32_t bitdivisor = int(FXOSC/bitrate);

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

  // Set Tx power
  LoRa.setTxPower(17);  // safe

  // Start RX mode
  LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

  // Set frequency
  setCtrlChan();

  // Dump registers
  dumpRegisters(ESP_LOG_DEBUG);

  radioLen = 0;
  LOGI("Receiver up and running");
}


// Call from Arduino loop to perform receiving tasks
void gtmlabLoop()
{
  char rx_byte = 0;
  uint8_t curr_ISR1=0;
  uint8_t curr_ISR2=0;

  // Read ISR registers once
  curr_ISR2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
  curr_ISR1 = LoRa.readRegister(REG_IRQ_FLAGS_1);

  if (curr_ISR1 & 0x02)  // Preamble detected, stay on channel
      scanning = false;

  if ((curr_ISR1 != prev_ISR1) || (curr_ISR2 != prev_ISR2)) {
    // Something changed in the ISR registers

    LOGV("IRQs: %02x|%02x", curr_ISR1, curr_ISR2);
    if (((prev_ISR1 & 0x02)!=0x02) && ((curr_ISR1 & 0x02)==0x02)) {
      LOGD("PREAMBLE (t=0)");
      pktStart=millis();
    }

    if (((prev_ISR1 & 0x03)!=0x03) && ((curr_ISR1 & 0x03)==0x03)) {
      LOGD("SYNCWORD (t=%d)", (millis()-pktStart));
      radioLen = 0;
      memset(radioBuf, 0, 256);
      scanning = false;   // Stay on channel
    }

    if (((prev_ISR2 & 0x04)!=0x04) && ((curr_ISR2 & 0x04)==0x04)) {
      LOGD("PAYREADY (t=%d)", (millis()-pktStart));
    }

    if (((prev_ISR1 & 0x03)==0x03) && ((curr_ISR1 & 0x03)!=0x03)) {
      if ((radioLen > 0) && !((curr_ISR2 & 0x04)==0x04)) {
        // lost sync while receiving packet - NOT GOOD
        LOGI("LOSTSYNC at pos %d/%d, t=%d", radioLen-1, radioBuf[0], (millis()-pktStart));
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
    // use PAYLOAD_READY ISR2 bit instead
    if ((curr_ISR2 & 0x04)==0x04) {
      // There may still be some unread payload in the fifo, so
      //   keep reading fifo until fifoempty
      while (!((curr_ISR2 = LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40)) {
        radioBuf[radioLen] = LoRa.readRegister(REG_FIFO);
        LOGD("< %02x", radioBuf[radioLen]);
        radioLen++;
      }

      LOGD("rxLen=%d (t=%d)", radioLen, (millis()-pktStart));

      ESP_LOG_BUFFER_HEXDUMP(TAG, radioBuf, radioLen, ESP_LOG_VERBOSE);
      if ((radioLen >= RX_MIN_LEN) && (radioLen <= RX_MAX_LEN)) {
        RS::ReedSolomon rs;
        rs.begin(radioLen-8, 1);
        if (rs.Decode(radioBuf, radioDec) == 0) {
          LOGD("REEDSOLO");
          // Packet OK, send it for further processing
          rxPacket(radioDec, radioLen-8);
        }
      } else {
        LOGD("Invalid rxLen");
        resetState();
      }

      pktStart = millis();
      radioLen = 0;
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


#include "playHarder.h"

void setup()
{
  Serial.begin(115200);     // initialize serial console
  while (!Serial);          // recommended if USB serial

  esp_log_level_set("*", VERBOSITY);

  // initialize radio and data structures
  gtmlabSetup();
}


void loop()
{
  // call radio 
  gtmlabLoop();

  // serial console input
  while (Serial.available()) {
    char conOne = Serial.read();
    if ((conOne == '\n') || (conLen>=(sizeof(conBuf)))){
      conBuf[conLen++] = 0;
      conExec();  // in playHarder.h
      conLen = 0;
    } else {
      conBuf[conLen++] = conOne;
    }
  }
}


// called from radio receiver for each good packet received
int rxPacket(uint8_t * rxBuf, uint8_t rxLen)
{
  ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, rxLen, ESP_LOG_VERBOSE);

  uint16_t msgH16;

  uint16_t crc_want = (rxBuf[rxLen-2]<<8) + rxBuf[rxLen-1];
  uint16_t crc_calc=0;
  for (int i=0; i<rxLen-2; i++) {
    crc_calc=CRC16_add(rxBuf[i], crc_calc);
  }
  crc_calc ^= 0xabcd;

  if (crc_want == crc_calc) {
    LOGD("CRC16-OK");
  } else {
    LOGI("CRC16BAD want:%04x, got:%04x", crc_want, crc_calc);
    return 0;
  }

  // good packet; drop the CRC
  rxLen-=2;

  // formatted channel information for debug output
  char chanDesc[16];
  if (recvData) {
    sprintf(chanDesc, "RX DCh=%02d", currChan);
  } else {
    sprintf(chanDesc, "RX CCh=%02d", currChan);
  }

  // First byte is raw packet length (ignorable by now)
  // Second byte is packet type
  if (rxBuf[1] == PKT_TYPE_SYNC) {
    // SYNC packet, indicates the begining of a data packet
    LOGI("%s SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
      chanDesc, rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5]);
      currDChIdx = rxBuf[2];
      wantFrags = rxBuf[3];
      // Save a copy
      lastIniTTL = rxBuf[4];
      lastCurTTL = rxBuf[5];
      LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xaa); // 2 bytes
      setDataChanX(++currDChIdx);
      dataStart = millis();
      recvData = true;

  } else if (rxBuf[1] == PKT_TYPE_ACK) {
    // ACK packet, indicates the successful delivery of a P2P message
    LOGI("%s ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d",
      chanDesc, (rxBuf[2]<<8)+rxBuf[3], 
      (rxBuf[4]>>4), (rxBuf[4] & 0x0f), rxBuf[5]);

    // output in standardized format
    printf("RX_ACK:");
    for (int i=2; i<rxLen; i++)
      printf("%02x", rxBuf[i]);
    printf("\n");
    resetState();

  } else if (rxBuf[1] == PKT_TYPE_DATA) {
    // DATA packet, a fragment of a protocol message of random length
    LOGI("%s DATA(2): len=%d, fragIDX=%d", chanDesc, rxBuf[2], rxBuf[3]);
    memcpy((packetBuf+packetLen), (rxBuf+4), rxBuf[2]);
    packetLen += rxBuf[2];
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
    LOGI("%s UNK (%d)", chanDesc, rxBuf[1]);    
  }
}


// Prepares the radio and state buffers for transmitting one or more packets
// There are a few things that need to be done before hitting TX on that radio
//   in order to make it a safe and seamless experience.
// This function performs all pre-flight steps, then sets the radio in TX mode
// The complement of this function, to be called at the END of the tx session,
//   is resetState(), which places the radio back in the default RX mode.
int txStart()
{
  // set mode sleep to prevent rx filling the fifo
  LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP);
  while (LoRa.readRegister(REG_IRQ_FLAGS_1));

  // Flush FIFO: read bytes until FIFOEMPTY
  // (is there any quicker way? do we need one?)
  while (!(LoRa.readRegister(REG_IRQ_FLAGS_2) & 0x40))
      LoRa.readRegister(REG_FIFO);

  // set TX mode, no need to wait to become ready
  LoRa.writeRegister(REG_OP_MODE, MODE_TX);  

  // set state flag
  inTXmode = true;

  return 0;
}


// Transmit a packet of size txLen, from txBuf
// The data is sent as-is; any envelope, correction codes etc are the 
//   responsibility of the caller
// The radio is expected to be in TX mode, tuned to chan, with an empty FIFO
//   (this is also the caller's job to ensure)
// The function finishes how it started: in TX mode with an empty FIFO
// NOTE: This function should probably not be used directly, but indirectly 
//   via txEncodeAndSend(), or one of the txSend(Ack|Sync|Msg) functions
int txPacket(uint8_t *txBuf, uint8_t txLen, bool isCtrl)
{
  uint8_t curr_ISR2=0;
  unsigned long txStart;  // millis when data TX started
  
  LOGD("TX START");
  txStart = millis();

  if (isCtrl) {
    // Control channel packets are 16 bytes fixed + 2 bytes syncword
    // We can fit up to (116-16-2)=98 bytes of preamble
    // ... start with 90 bytes and work from there
    LOGD("LONG_PRE");
    LoRa.writeRegister(REG_FSK_PREAMBLE_LSB, 90);
  } else {
    // Data channel packets are up to 104 bytes + 2 bytes syncword
    // Use 10 bytes of preamble
    LOGD("SHORT_PRE");
    LoRa.writeRegister(REG_FSK_PREAMBLE_LSB, 10);
  }

  uint8_t fifoLevel = 0;
  for (int i=0; i<txLen; i++) {
    LoRa.writeRegister(REG_FIFO, txBuf[i]);
    fifoLevel++;

    // When our FIFO load level near maximum,
    // to avoid an overrun, stop loading any more bytes until
    //   FIFO level drops below threshold
    while(fifoLevel>=64) { // FIXME fifosize
      // read ISR until "fifo above threshold" bit clears
      curr_ISR2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
      if(curr_ISR2 & 0x20) {  // fifolevel
        LOGV("ISR2=%02x", curr_ISR2);
      } else {
        fifoLevel = 15;  // FIXME fifothre
      }
    }
  }

  // Finished uploading the data, however, at this point, 
  //   the radio is still transmitting from the fifo;
  // wait for PacketSent bit to confirm transmission complete
  while(!(LoRa.readRegister(REG_IRQ_FLAGS_2) & 0x08));
  LOGD("TX DONE, t=%dms", (millis() - txStart));
}


// Takes a packet payload,
// prepends packet header, appends crc16 and reed-solomon code
// and then invokes txPacket() to SEND the packet over radio
// NOTE: Don't use this function directly unless you really need to
//  the txSend(Ack|Sync|Msg) functions below are easier and safer
int txEncodeAndSend(uint8_t * pktBuf, uint8_t pktLen, uint8_t pktType)
{
  bool isCtrl = false;   // is it a control packet?

  LOGV("PAY_SIZE=%d", pktLen);

  // First byte is the output packet length (not including itself)
  // = size + typebyte + crc16 + reedsolo
  radioBuf[0] = pktLen + 1 + 2 + 8;

  // Second byte is packet type
  radioBuf[1] = pktType;

  // Then the actual packet bytes
  memcpy(radioBuf+2, pktBuf, pktLen);

  // Calculate CRC16 on what we got so far
  uint16_t crc_calc=0;
  for (int i = 0; i < (pktLen+2); i++) {
    crc_calc=CRC16_add(radioBuf[i], crc_calc);
  }
  crc_calc ^= 0xabcd;

  // Append CRC16 at end of packet
  radioBuf[pktLen+2] = (crc_calc >> 8);
  radioBuf[pktLen+3] = (crc_calc & 0xff);

  // count length and type bytes and CRC16
  pktLen += 4;

  RS::ReedSolomon rs;
  // reedsolo encode packet
  // since the encoding only appends bytes at the end,
  //   it's safe to use same output buffer as input
  rs.begin(pktLen, 1);
  rs.Encode(radioBuf, radioBuf);

  // SYNC and ACK are control packets (require long preamble)
  if ((pktType == PKT_TYPE_SYNC) || (pktType == PKT_TYPE_ACK)) {
    isCtrl = true;
  }

  LOGV("AIR_SIZE: %d", pktLen + 8);

  // and finally transmit outbuf
  txPacket(radioBuf, pktLen + 8, isCtrl);
}


// Send a SYNC packet 
// (will set channel; expects TX mode on)
int txSendSync(uint8_t chIDX, uint8_t frags, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = chIDX;
  mBuf[1] = frags;
  mBuf[2] = iniTTL;
  mBuf[3] = curTTL;
  
  setCtrlChan();  // jump to a control channel

  LOGI("TX CCh=%02d SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
        currChan, chIDX, frags, iniTTL, curTTL);
  txEncodeAndSend(mBuf, 4, PKT_TYPE_SYNC);
}


// Send an ACK packet
// (will set channel; expects TX mode on)
int txSendAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = (hashID >> 8);
  mBuf[1] = (hashID & 0xff);
  mBuf[2] = (hops & 0x0f)<<4;
  mBuf[2] |= (iniTTL & 0x0f);
  mBuf[3] = curTTL;

  setCtrlChan();  // jump to a control channel

  LOGI("TX CCh=%02d ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d", 
        currChan, hashID, hops, iniTTL, curTTL);
  txEncodeAndSend(mBuf, 4, PKT_TYPE_ACK);
}


// Send a message
// This is a complex action involving several packets
// (will set channel; expects TX mode on)
int txSendMsg(uint8_t * mBuf, uint8_t mLen, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t oBuf[128];   // output buffer
  uint8_t oLen = 0;    // output length

  uint8_t nFrags = ceil(1.0 * mLen / TXFRAGSIZE);

  // send sync packet
  txSendSync(currDChIdx, nFrags, iniTTL, curTTL);

  // allow some time for receivers to switch channel
  // from experiments, a value of 10ms works well here
  delay(txSyncDelay);

  for (int i=0; i<nFrags; i++) {
    int mOfs = i * TXFRAGSIZE;
    if ((mLen - mOfs) >= TXFRAGSIZE) {
      oLen = TXFRAGSIZE;
    } else {
      oLen = mLen % TXFRAGSIZE;
    }

    // Fragment prefix: two bytes
    oBuf[0] = oLen;  // length of fragment
    oBuf[1] = i;     // fragment index

    // fill fragment buffer with data
    memcpy(oBuf + 2, mBuf + mOfs, oLen);
    oLen += 2;  // count two bytes at beginning

    // hop to next channel in chan map
    currDChIdx++;
    currDChIdx %= sizeof(dataChans);
    setDataChanX(currDChIdx);

    // send the fragment
    txEncodeAndSend(oBuf, oLen, PKT_TYPE_DATA);

    // output after data upload, to avoid any uncontrolled delays
    LOGI("TX DCh=%02d DATA(2): len=%d, fragIDX=%d", currChan, oLen-2, i);

    while (!LoRa.readRegister(REG_IRQ_FLAGS_2) & 0x40);  // wait for FifoEmpty
    delay(txPackDelay);  // wait for others
  }
}
