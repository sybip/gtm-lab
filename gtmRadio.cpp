//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include <SPI.h>

#include "gtmConfig.h"
#include "LoRaX.h"
#include "LoRaRegs.h"
#include "RS-mod-FCR.h"
#include "gtmRadio.h"

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

// Logging functions and options
#define TAG "GTMLIB"

regSet regSets[] = {
  // REGION=US
  { 902500000, 500000, 
    3, { 1, 25, 49 },
    48, { 7, 42, 41, 46, 48, 45, 11, 18, \
          6, 2, 26, 13, 19, 39, 37, 43, \
          10, 15, 20, 50, 16, 21, 32, 22, \
          23, 0, 40, 9, 35, 33, 28, 34, \
          24, 31, 17, 30, 27, 44, 14, 4, \
          3, 29, 38, 36, 5, 12, 8, 47 },
  },

  // REGION=EU
  { 869425000, 50000,
    2, { 0, 4 },
    3, { 1, 2, 3 },
  },

  // REGION=AU
  { 915500000, 500000,
    3, { 1, 12, 23 },
    22, { 7, 11, 18, 6, 2, 13, 19, 10, \
          15, 20, 16, 21, 22, 0, 9, 24, \
          17, 14, 4, 3, 5, 8 },
  },

  // REGION=JP
  { 920500000, 500000,
    2, { 0, 8 },
    7, { 7, 6, 2, 1, 4, 3, 5 },
  }

};

// FREQUENCY BAND SETTINGS
#if ISM_REGION==1     // REGION=US
#define REGIX 0

#elif ISM_REGION==2   // REGION=EU
#define REGIX 1

#elif ISM_REGION==4   // REGION=AU
#define REGIX 2

#elif ISM_REGION==8   // REGION=JP
#define REGIX 3

#else
#error "Invalid ISM_REGION"
#endif

// Current regset - can be changed at runtime
regSet * curRegSet = &(regSets[REGIX]);

// Common settings for GTM waveform
int bitrate = 24000;
int freqdev = 12500;

// Some constants related to radio timings etc,
// Determined mostly through trial-and-error
#define PKT_TIMEOUT 40 // millis
#define PRE_TIMEOUT 20  // data channel preable timeout
#define CCH_SILENCE 40 // millis
#define SCAN_DWELL 10  // millis
// FIFO threshold must be ONE LESS than the smallest TX packet
//   otherwise TX will not start, causing issue #2
#define FIFOTHRE 14  // smallest packet is 1-byte DATA (air size=15)
#define FIFOSIZE 64  // actually 66, but leave a margin
uint8_t txSyncDelay = 10;  // millis to wait between sync packet and first data packet
uint8_t txPackDelay = 8;   // millis to wait between data packets
uint8_t TXFRAGSIZE = 90;   // it's what they use; up to 96 accepted, 98 with errors
// min and max acceptable RX packet sizes (including RS ECC)
#define RX_MIN_LEN 10   // min
#define RX_MAX_LEN 200  // max

// message hash ringbuffer size, for duplicate detection
// there will be 3 of these: ACK RX, Msg RX and Msg TX
#define HASH_BUF_LEN 64

#define FXOSC 32E6  // 32MHz

// Data buffers
uint8_t radioBuf[256];    // radio packet buffer
uint8_t radioDec[248];    // reedsolo decode buffer
uint8_t packetBuf[512];   // "a CVE waiting to happen"
uint16_t packetLen = 0;
int radioLen = 0;
int wantLen = 0;

// Msg hash ringbuffers
uint16_t rxAckBuf[HASH_BUF_LEN] = { 0 };
uint16_t rxMsgBuf[HASH_BUF_LEN] = { 0 };
uint16_t txMsgBuf[HASH_BUF_LEN] = { 0 };
uint8_t rxAckPos = 0;
uint8_t rxMsgPos = 0;
uint8_t txMsgPos = 0;

// Useful values from last sync packet
uint8_t wantFrags = 0; // Number of fragments we expect
uint8_t dataFrags = 0; // Number of fragments we expect
uint8_t lastIniTTL;  // for standardized output
uint8_t lastCurTTL;  // for standardized output

// Channel hopping related variables
bool scanning = true;
bool holdchan = false;  // if true, stay on this chan
bool inTXmode = false;
bool recvData = false;  // if true, we are on a data chan
uint8_t currChan = 0;   // current channel NUMBER
uint8_t currDChIdx = 0; // current data chan INDEX in map
uint8_t currCChIdx = 0; // current ctrl chan INDEX in map
// millis when last preamble detected on cChan
unsigned long lastCChAct[4] = { 0, 0, 0, 0 };

bool relaying = DFLT_RELAY;  // enable mesh relay function

// Timing related variables
esp_log_level_t logLevel = VERBOSITY;  // in case we want to query it
unsigned long pktStart = 0;   // millis when packet RX started
unsigned long dataStart = 0;  // millis when data RX started
unsigned long lastHop = 0;    // millis when we last hopped ctrl chan
unsigned long chanTimer = 0;  // millis when we entered current channel

// RSSI, keep it simple
uint8_t pktRSSI = 0;  // absolute value; RSSI = -RssiValue/2[dBm]
uint8_t cChRSSI = 0;  // control chan RSSI (for LBT)
uint16_t sumDChRSSI = 0;  // sum of data channel RSSI, for averaging

// Interrupt Status Registers - stored previous values
uint8_t prev_IRQ1 = 0, prev_IRQ2 = 0;

// short-term buffers for queuing outbound messages 
//   while waiting for a clear channel to transmit
// buffer sizes are unscientific broad estimates
#define MSG_QUEUE_SIZE 6
#define ACK_QUEUE_SIZE 6

struct msgDesc {
  uint8_t iniTTL;
  uint8_t curTTL;
  uint16_t hashID;
  uint16_t msgLen;
  uint8_t msgObj[500];  // FIXME explain
};

struct ackDesc {
  uint8_t iniTTL;
  uint8_t curTTL;
  uint16_t hashID;
  uint8_t hops;
};

msgDesc msgQueue[MSG_QUEUE_SIZE];
ackDesc ackQueue[ACK_QUEUE_SIZE];

uint8_t msgQHead = 0, msgQTail = 0;
uint8_t ackQHead = 0, ackQTail = 0;

// number of calls to gtmLabLoop() per received packet
// (it's currently not optimal, we need to bring it way down)
int pktLoops = 0;    // 20210303 PKTLOOPS


// user-definable event handler callbacks
// MSG rx handler
bool (* onRxMSG)(uint8_t *, uint16_t, uint8_t, uint8_t, uint8_t) = builtinRxMSG;
// ACK rx handler
bool (* onRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t) = builtinRxACK;


// additive CRC16 function
uint16_t CRC16_add(uint8_t b, uint16_t crc)
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
    for (int i = 0; i<len; i++) {
        x = (((x + str[i]) * mult + incr) & 0xFFFFFFFF) % modulus;
        h ^= x;
    }

    // Derive 16-bit value from 32-bit hash by XORing its two halves
    return ((h & 0xFFFF0000) >> 16) ^ (h & 0xFFFF);
}


// calculates message hash using GTH16 algo
uint16_t msgHash16(uint8_t* mBuf)
{
  // position of HEAD element (depends on message class)
  uint8_t headPos = 15;

  if ((mBuf[0] == MSG_CLASS_SHOUT) || (mBuf[0] == MSG_CLASS_EMERG)) {
    headPos = 5;
  }

  // FIXME - do a sanity check on TLV header? (FB 10)

  return gtAlgoH16(mBuf+headPos, 16);
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
      LOG_(logLevel, TAG, 
        "0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN, i, rVal, BYTE_TO_BINARY(rVal));      
    }
  }
}


// Look up a hash in one of the hash ringbuffers
bool inRingBuf(uint16_t needle, uint16_t *haystack)
{
  for (int i = 0; i < HASH_BUF_LEN; i++) {
    if (haystack[i] == needle)
      return true;
  }
  return false;
}


// Set frequency, argument is channel number (0 to NUMCHANS)
void setChan(uint8_t chan)
{
  unsigned long tStart = millis();

  LoRa.setFrequency(curRegSet->baseFreq + (chan*curRegSet->chanStep));

  bool locked = false;
  while(!locked) {
    uint8_t curr_IRQ1 = LoRa.readRegister(REG_IRQ_FLAGS_1);
    // MODEREADY | PLLLOCK = 0b10010000
    if ((curr_IRQ1 & 0x90) == 0x90)
      locked = true;
  }

  chanTimer = millis();

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
  dChanX %= curRegSet->dChanNum;
  setChan(curRegSet->dChanMap[dChanX]);
}

// Tune to a control channel (next in sequence and increment)
// Call this repeatedly to traverse channel map in a scanning pattern
void setCtrlChan()
{
  currCChIdx++;
  currCChIdx %= curRegSet->cChanNum;
  setChan(curRegSet->cChanMap[currCChIdx]);
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
  pktStart = 0;   // we're not receiving a packet

  // initialize channel activity timers
  for (int i=0; i<4; i++)
    lastCChAct[i] = millis();

  // reset the radio's FIFO
  while (!((LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40))
    LoRa.readRegister(REG_FIFO);

  if (!holdchan)
    setCtrlChan();
}


// Call from Arduino setup() to initialize radio and data structures
void gtmlabInit()
{
#ifdef PIN_SCLK
  // radio sits on a non-default SPI
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
#endif

  uint32_t bitdivisor = int(FXOSC/bitrate);

  LOGI("Receiver startup: board=%d, region=%d", BOARD_TYPE, ISM_REGION);

  // Configure the CS, reset, and IRQ pins
  LoRa.setPins(pinCS, pinRST, pinIRQ);

  if (!LoRa.begin(curRegSet->baseFreq)) {   // init radio at base frequency
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

  // Set FIFO threshold
  LoRa.writeRegister(REG_FIFO_THRESHOLD, FIFOTHRE);

  // RegPLL Hop - set fast hopping
  LoRa.writeRegister(REG_PLL_HOP, (0x2d | 0x80));

  // Set Tx power
  LoRa.setTxPower(DFLT_POWER);  // defined in gtmConfig.h

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
  uint8_t curr_IRQ1 = 0;
  uint8_t curr_IRQ2 = 0;
  bool payReady = false;

  pktLoops++;    // 20210303 PKTLOOPS

  // Read IRQ registers once
  curr_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
  curr_IRQ1 = LoRa.readRegister(REG_IRQ_FLAGS_1);

  if ((curr_IRQ1 != prev_IRQ1) || (curr_IRQ2 != prev_IRQ2)) {
    // Something changed in the IRQ registers

    LOGV("IRQs: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" %s:%s:%s", 
             BYTE_TO_BINARY(curr_IRQ1),
             BYTE_TO_BINARY(curr_IRQ2),
          (curr_IRQ1 & 0x02) ? "PR":"--",
          (curr_IRQ1 & 0x01) ? "SW":"--",
          (curr_IRQ2 & 0x04) ? "PL":"--");          

    // First check preamble, then syncword, then payready
    // The order is IMPORTANT - that's why we need a FSM

    if (((prev_IRQ1 & 0x02)!=0x02) && ((curr_IRQ1 & 0x02)==0x02)) {
      LOGD("PREAMBLE (t=0)");
      pktStart=millis();
      pktLoops = 0;    // 20210303 PKTLOOPS
      scanning = false;
      if (!recvData) {
        // record "last preamble detected" for control channel
        lastCChAct[currCChIdx] = millis();
      }
    }

    if (((prev_IRQ1 & 0x03)!=0x03) && ((curr_IRQ1 & 0x03)==0x03)) {
      if (! pktStart) {
        // missed the preamble, but it's OK, start the timer now
        pktStart=millis();
        pktLoops = 0;    // 20210303 PKTLOOPS
      }
      LOGD("PRE+SYNW (t=%d)", (millis()-pktStart));
      radioLen = 0;
      memset(radioBuf, 0, 256);
      scanning = false;   // Stay on channel
    }

    if (((prev_IRQ2 & 0x04)!=0x04) && ((curr_IRQ2 & 0x04)==0x04)) {
      if (pktStart) {
        LOGD("PAYREADY (t=%d)", (millis()-pktStart));
        payReady = true;
      } else {
        LOGD("PAYREADY IGNORED");  // likely a misfire
        resetState();
        // FIXME - bailout?
      }
    }

    if (((prev_IRQ1 & 0x03)==0x03) && ((curr_IRQ1 & 0x03)!=0x03)) {
      if ((radioLen > 0) && !((curr_IRQ2 & 0x04)==0x04)) {
        // lost sync while receiving packet - NOT GOOD
        // Read RSSI (debug purposes only)
        pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
        LOGI("LOSTSYNC %cCh=%02d pos %d/%d, RSSI=-%d t=%d", 
              holdchan ? '*' : (recvData ? 'D':'C'), 
              currChan, 
              radioLen-1, radioBuf[0], 
              pktRSSI>>1, (millis()-pktStart));

        // From practical observations, LOSTSYNC on a control chan
        //   usually indicates a bad "packet size" byte, which is 
        //   a hard one to recover from.
        // Abandon this packet and get ready for next one ASAP
        if ((!recvData) && radioBuf[0] > 16) {  // normal size=15
          resetState();
          return;  // BAILOUT
        }
      }
    }

    // store values as "previous"
    prev_IRQ1=curr_IRQ1;
    prev_IRQ2=curr_IRQ2;
  }

  if(((curr_IRQ1 & 0x03)==0x03) && 
     ((curr_IRQ2 & 0x20) || !(curr_IRQ2 & 0x40))) {

    if (curr_IRQ2 & 0x20) {
      // FIFO above threshold, read FIFOTHRE bytes blindly
      for (int i=0; i<FIFOTHRE; i++) {
        radioBuf[radioLen++] = LoRa.readRegister(REG_FIFO);
      }
    } else {
      // FIFO below threshold, read one-by-one until empty
      while (!((curr_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40)) {
        radioBuf[radioLen++] = LoRa.readRegister(REG_FIFO);

        // keep watch for PAYREADY signaling at all times
        if (curr_IRQ2 & 0x04) {
          payReady = true;
        }
      }
    }    

    // check PAYLOAD_READY IRQ2 bit (read and saved above)
    //if (payReady || (curr_IRQ2 & 0x04)) {
    if (payReady) {
      // Read RSSI ASAP
      pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);

      // LOGD("rxLen=%d, RSSI=-%d (t=%d)", radioLen, (pktRSSI>>1), (millis()-pktStart));
      // 20210303 PKTLOOPS
      LOGD("rxLen=%d, RSSI=-%d (t=%d, loop=%d)", radioLen, (pktRSSI>>1), (millis()-pktStart), pktLoops);
      pktLoops = 0;  // 20210303 PKTLOOPS

      HEXV(TAG, radioBuf, radioLen);
      if ((radioLen >= RX_MIN_LEN) && (radioLen <= RX_MAX_LEN)) {
        RS::ReedSolomon rs;
        rs.begin(radioLen-8, 1);
        if (rs.Decode(radioBuf, radioDec) == 0) {
          LOGD("REEDSOLO");
          // Packet OK, send it for further processing
          rxPacket(radioDec, radioLen-8, pktRSSI>>1);
        } else {
          LOGD("REEDSOLO FAIL");  // bail out
          resetState();
          return;
        }
      } else {
        LOGD("Invalid rxLen");
        resetState();
        return;
      }

      pktStart = 0;  // no packet being received right now
      radioLen = 0;
    }
  }

  // in recvData mode, we just jumped on a data chan and are 
  //   currently waiting for a preamble?
  // if we missed the preamble, make sure we don't wait forever
  if (recvData && (! (curr_IRQ1 & 0x02)) && 
      (millis() - chanTimer > PRE_TIMEOUT)) {
    pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
    LOGI("PRESTALL %cCh=%02d, RSSI=-%d (t=%d)",
          holdchan ? '*' : (recvData ? 'D':'C'), 
          currChan,
          pktRSSI>>1, millis()-chanTimer);
    resetState();   // BAILOUT
    return;
  }

  // flexible packet timeout: if in verbose mode, allow
  //   some extra time for the serial output (up to 50 ms)
#define FLEX_TIMEOUT ((logLevel < ESP_LOG_VERBOSE) ? PKT_TIMEOUT : 50)

  // If a packet reception doesn't complete in this time, 
  //  it never will; reset state immediately to hopefully
  //  pick up a retransmission
  if ((pktStart > 0) && (millis()-pktStart > FLEX_TIMEOUT)) {
    // Read RSSI (debug purposes only)
    pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
    LOGI("PKTSTALL %cCh=%02d pos %d/%d, RSSI=-%d (t=%d)",
          holdchan ? '*' : (recvData ? 'D':'C'), 
          currChan, 
          radioLen-1, radioBuf[0], 
          pktRSSI>>1, millis()-pktStart);
    resetState();
    return;
  }

  if (scanning && !recvData) {
    // if scanning, hop to next chan every SCAN_DWELL millis
    if ((millis()-lastHop) > SCAN_DWELL) {
      cChRSSI = LoRa.readRegister(REG_RSSI_VALUE);
      // At this point, we're on a control channel that's not receiving anything;
      // read RSSI, and if channel clear, STAY ON CHANNEL and
      // send any queued transmission
      if (cChRSSI > 160) {  // silence level -80dBi, FIXME explain this
          if (msgQueue[msgQTail].curTTL > 0) {
            LOGI("CCh=%d(%d), RSSI=-%d, can TX MSG(pos:%d)", 
                 currCChIdx, currChan, (cChRSSI>>1), msgQTail);
            txStart();
            txSendMsg(msgQueue[msgQTail].msgObj, msgQueue[msgQTail].msgLen, 
                      msgQueue[msgQTail].iniTTL, msgQueue[msgQTail].curTTL);
            memset(&(msgQueue[msgQTail]), 0, sizeof(struct msgDesc));
            msgQTail = (msgQTail+1) % MSG_QUEUE_SIZE;
            resetState();
          } else if (ackQueue[ackQTail].curTTL > 0) {
            LOGI("CCh=%d(%d), RSSI=-%d, can TX ACK(pos:%d)", 
                 currCChIdx, currChan, (cChRSSI>>1), ackQTail);
            txStart();
            txSendAck(ackQueue[ackQTail].hashID, ackQueue[ackQTail].hops, 
                      ackQueue[ackQTail].iniTTL, ackQueue[ackQTail].curTTL);
            memset(&(ackQueue[ackQTail]), 0, sizeof(struct ackDesc));
            ackQTail = (ackQTail+1) % ACK_QUEUE_SIZE;
            resetState();
          }
      }
      // otherwise, just HOP
      if (!holdchan) {
        setCtrlChan();
        lastHop = millis();
      }
    }
  }
}


// called from radio receiver for each good packet received
//   (uRSSI = unsigned RSSI)
int rxPacket(uint8_t * rxBuf, uint8_t rxLen, uint8_t uRSSI)
{
  // this is redundant
  // HEXV(TAG, rxBuf, rxLen);

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
  // DCh = data, CCh = ctrl, *Ch = user entered
  char chanDesc[16];
  sprintf(chanDesc, "RX %cCh=%02d", 
          holdchan ? '*' : (recvData ? 'D':'C'), 
          currChan);

  // First byte is raw packet length (ignorable by now)
  // Second byte is packet type
  if (rxBuf[1] == PKT_TYPE_SYNC) {
    // SYNC packet, indicates the begining of a data packet
    LOGI("%s SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
      chanDesc, rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5]);

      // hop to receive data (unless channel hold is engaged)
      if (!holdchan) {
        currDChIdx = rxBuf[2];
        dataFrags = wantFrags = rxBuf[3];
        // Save a copy
        lastIniTTL = rxBuf[4];
        lastCurTTL = rxBuf[5];
        sumDChRSSI = 0;  // reset sum
        //
        LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xaa); // 2 bytes
        setDataChanX(++currDChIdx);
        dataStart = millis();
        recvData = true;
      }

  } else if (rxBuf[1] == PKT_TYPE_TIME) {
    // TIME packet, log for now but don't act
    uint32_t tStamp = (rxBuf[2]<<24)+(rxBuf[3]<<16)+(rxBuf[4]<<8)+rxBuf[5];
    LOGI("%s TIME(0): time=0x%08x", chanDesc, tStamp);

  } else if (rxBuf[1] == PKT_TYPE_ACK) {
    // ACK packet, indicates the successful delivery of a P2P message
    msgH16 = (rxBuf[2]<<8)+rxBuf[3];
    LOGI("%s ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d",
      chanDesc, msgH16, 
      (rxBuf[4]>>4), (rxBuf[4] & 0x0f), rxBuf[5]);

    if (inRingBuf(msgH16, rxAckBuf)) {
      LOGI("ACK SEEN BEFORE");
    } else {
      rxAckBuf[rxAckPos++] = msgH16;
      rxAckPos %= HASH_BUF_LEN;

      // what's this ACK for?
      if (inRingBuf(msgH16, txMsgBuf)) {
        LOGI("ACK for TXd MSG");
      } else if (inRingBuf(msgH16, rxMsgBuf)) {
        LOGI("ACK for RXd MSG");
      } else {
        LOGI("ACK without MSG");        
      }

      // if first seen, and curTTL>1, it's a relayable ACK
      if (rxBuf[5]>1) {
        ////////// RELAYING //////////
        LOGI("RELAYACK (%s)", (relaying ? "ON!":"OFF"));
        if (relaying) {
          // decrement curTTL and enqueue for TX
          // FIXME catch error
          txEnQueueACK(msgH16, (rxBuf[4]>>4), (rxBuf[4] & 0x0f), (rxBuf[5]-1));
        }
      }

      // new ACK received - call external handler if defined
      if ((* onRxACK) != nullptr) {
        if (! onRxACK(msgH16, (rxBuf[4]>>4), (rxBuf[4] & 0x0f), rxBuf[5], pktRSSI)) {
          LOGD("ACK handler failed");
        }
      }
    }
    resetState();

  } else if (rxBuf[1] == PKT_TYPE_DATA) {
    // DATA packet, a fragment of a protocol message of random length
    LOGI("%s DATA(2): len=%d, fragIDX=%d", chanDesc, rxBuf[2], rxBuf[3]);
    memcpy((packetBuf+packetLen), (rxBuf+4), rxBuf[2]);
    packetLen += rxBuf[2];
    wantFrags--;

    sumDChRSSI += uRSSI;

    if (wantFrags) {
      // hop to next data channel to receive more fragments
      if (!holdchan) {
        setDataChanX(++currDChIdx);
      }

    } else {
      // no more frags; output packet and hop to control channel
      // ...but first, calculate GTH16 hash
      msgH16 = msgHash16(packetBuf);
      LOGI("RX complete: len=%d, hash=0x%04x, time=%dms, RSSI=-%d", 
           packetLen, msgH16, (millis()-dataStart), 
           int(sumDChRSSI/(double)dataFrags));

      if (inRingBuf(msgH16, rxMsgBuf)) {
        LOGI("MSG SEEN BEFORE");
      } else {
        rxMsgBuf[rxMsgPos++] = msgH16;
        rxMsgPos %= HASH_BUF_LEN;

        // New message received - call external handler if defined
        if ((* onRxMSG) != nullptr) {
          if (! onRxMSG(packetBuf, packetLen, lastIniTTL, lastCurTTL, 
                        int(sumDChRSSI/(double)dataFrags))) {
            LOGD("MSG handler failed");
          }
        }

        // Is this an echo of a message that we sent?
        if (inRingBuf(msgH16, txMsgBuf)) {
          LOGI("MSG SENT BEFORE");
        } else {
          // if first seen, and curTTL>1, it's a relayable message
          if (lastCurTTL>1) {
            ////////// RELAYING //////////
            LOGI("RELAYMSG (%s)", (relaying ? "ON!":"OFF"));
            if (relaying) {
              // decrement curTTL and enqueue for TX
              // FIXME catch error
              txEnQueueMSG(packetBuf, packetLen, lastIniTTL, (lastCurTTL-1));
            }
          }
        }
      }
      packetLen=0;
      // will reset buffers, hop to cch and start scanning
      resetState();
    }

  } else {
    LOGI("%s UNK (%d)", chanDesc, rxBuf[1]);    
  }
}


// simple builtin MSG handler - hexlified output
// (uRSSI because the RSSI is unsigned)
bool builtinRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI)
{
  // output in standardized format (inittl, curttl, uRSSI, | ,hexmsg)
  printf("RX_MSG:");
  printf("%02x%02x%02x|", iniTTL, curTTL, uRSSI);
  for (int i=0; i<mLen; i++)
    printf("%02x", mBuf[i]);
  printf("\n");
  return true;
}


// simple builtin ACK handler - hexlified output
bool builtinRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI)
{
  // output in standardized format
  printf("RX_ACK:%04x%02x%02x%02x\n", hashID, ((hops & 0x0f)<<4) | (iniTTL & 0x0f), curTTL, uRSSI);
  return true;  
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


// WARNING: txPacket, txEncodeAndSend and txSend(Ack|Sync|Msg|Time) are
//  not collision-resistant; production applications should use the
//  txEnQueue* functions instead

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
  uint8_t curr_IRQ2=0;
  unsigned long txStart;  // millis when data TX started

  HEXV(TAG, txBuf, txLen);

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
    while(fifoLevel >= FIFOSIZE) {
      // read IRQ until "fifo above threshold" bit clears
      curr_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
      if(curr_IRQ2 & 0x20) {  // fifolevel
        LOGV("IRQ2=%02x", curr_IRQ2);
      } else {
        fifoLevel = FIFOTHRE;
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
// NOTE: Don't use this function directly unless you really need to;
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

  // SYNC, ACK and TIME are control packets (require long preamble)
  if (pktType != PKT_TYPE_DATA) {
    isCtrl = true;
  }

  LOGV("AIR_SIZE: %d", pktLen + 8);

  // and finally transmit outbuf
  txPacket(radioBuf, pktLen + 8, isCtrl);
}


// Send a SYNC packet (will set channel; expects TX mode on)
int txSendSync(uint8_t chIDX, uint8_t frags, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = chIDX;
  mBuf[1] = frags;
  mBuf[2] = iniTTL;
  mBuf[3] = curTTL;
  
  // CCH already set from main loop, just stay on it
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
        currChan, chIDX, frags, iniTTL, curTTL);
  txEncodeAndSend(mBuf, 4, PKT_TYPE_SYNC);
}


// Send an ACK packet (will set channel; expects TX mode on)
int txSendAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = (hashID >> 8);
  mBuf[1] = (hashID & 0xff);
  mBuf[2] = (hops & 0x0f)<<4;
  mBuf[2] |= (iniTTL & 0x0f);
  mBuf[3] = curTTL;

  // setCtrlChanTX();  // jump to a FREE control channel
  // CCH already set from main loop, just stay on it
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d", 
        currChan, hashID, hops, iniTTL, curTTL);
  txEncodeAndSend(mBuf, 4, PKT_TYPE_ACK);
}


// Send a TIME packet (will set channel; expects TX mode on)
int txSendTime(uint64_t time32)
{
  uint8_t mBuf[4];
  // timestamp
  mBuf[0] = (time32 >> 24) & 0xff;
  mBuf[1] = (time32 >> 16) & 0xff;
  mBuf[2] = (time32 >> 8) & 0xff;
  mBuf[3] = time32 & 0xff;

  // CCH already set from main loop, just stay on it
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d TIME(0): 0x%08x", currChan, time32);
  txEncodeAndSend(mBuf, 4, PKT_TYPE_TIME);
}



// Send a message
// This is a complex action involving several packets
// (will set channel; expects TX mode on)
int txSendMsg(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t oBuf[128];   // output buffer
  uint8_t oLen = 0;    // output length

  uint8_t nFrags = ceil(1.0 * mLen / TXFRAGSIZE);

  // if channel hold engaged, we can't transmit, so bail out
  if (holdchan) {
    LOGW("TX disabled due to channel hold");
    return -1;
  }

  // calculate message hash
  uint16_t msgH16 = msgHash16(mBuf);
  LOGI("TX start: len=%d, hash=0x%04x, frags=%d", mLen, msgH16, nFrags);

  // save to ringbuffer
  txMsgBuf[txMsgPos++] = msgH16;
  txMsgPos %= HASH_BUF_LEN;

  // pick a random data channel
  currDChIdx = random(curRegSet->dChanNum - 1);

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
    currDChIdx %= curRegSet->dChanNum;
    setDataChanX(currDChIdx);

    // send the fragment
    txEncodeAndSend(oBuf, oLen, PKT_TYPE_DATA);

    // output after data upload, to avoid any uncontrolled delays
    LOGI("TX DCh=%02d DATA(2): len=%d, fragIDX=%d", currChan, oLen-2, i);

    while (!LoRa.readRegister(REG_IRQ_FLAGS_2) & 0x40);  // wait for FifoEmpty
    delay(txPackDelay);  // wait for others
  }
}


// Queue a message for sending (when channel is clear)
bool txEnQueueMSG(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL)
{
  if (msgQueue[msgQHead].curTTL>0) {
    // buffer is full, retry later
    return false;
  }
  msgQueue[msgQHead].iniTTL = iniTTL;
  msgQueue[msgQHead].curTTL = curTTL;
  msgQueue[msgQHead].msgLen = msgLen;
  memcpy(msgQueue[msgQHead].msgObj, msgObj, msgLen);
  msgQHead = (msgQHead+1) % MSG_QUEUE_SIZE;

  return true;
}


// Queue an ACK for sending (when channel is clear)
bool txEnQueueACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  if (msgQueue[msgQHead].curTTL>0) {
    // buffer is full, retry later
    return false;
  }
  ackQueue[ackQHead].iniTTL = iniTTL;
  ackQueue[ackQHead].curTTL = curTTL;
  ackQueue[ackQHead].hashID = hashID;
  ackQueue[ackQHead].hops = hops;
  ackQHead = (ackQHead+1) % ACK_QUEUE_SIZE;

  return true;
}
