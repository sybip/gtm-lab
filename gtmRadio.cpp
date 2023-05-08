//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
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

#elif BOARD_TYPE==2
// Settings for Sparkfun WRL-15006 (ESP32 Lora Gateway)
#define pinCS 16  // LoRa radio chip select
#define pinRST 27  // LoRa radio reset
#define pinIRQ 26  // Hardware interrupt pin

#elif BOARD_TYPE==3
// Sparkfun Micromod, LoRa 1W board at function zero
#define pinCS 5   // LoRa radio chip select
#define pinRST 25  // LoRa radio reset
#define pinIRQ 14  // Hardware interrupt pin

#else
#error "Invalid BOARD_TYPE"
#endif

// Logging functions and options
#define TAG "GTMLIB"

regSet regSets[] = {
  // REGION=US
  { 902500000, 500000, 51,
    3, { 1, 25, 49 },
    48, { 7, 42, 41, 46, 48, 45, 11, 18, \
          6, 2, 26, 13, 19, 39, 37, 43, \
          10, 15, 20, 50, 16, 21, 32, 22, \
          23, 0, 40, 9, 35, 33, 28, 34, \
          24, 31, 17, 30, 27, 44, 14, 4, \
          3, 29, 38, 36, 5, 12, 8, 47 },
  },

  // REGION=EU
  { 869425000, 50000, 5,
    2, { 0, 4 },
    3, { 1, 2, 3 },
  },

  // REGION=AU
  { 915500000, 500000, 25,
    3, { 1, 12, 23 },
    22, { 7, 11, 18, 6, 2, 13, 19, 10, \
          15, 20, 16, 21, 22, 0, 9, 24, \
          17, 14, 4, 3, 5, 8 },
  },

  // REGION=JP
  { 920500000, 500000, 9,
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
#define PKT_TIMEOUT 42 // millis
#define PRE_TIMEOUT 20  // data channel preamble timeout
#define LBT_SILENCE 10 // millis to listen for silence before tx
#define LBT_MAX_HITS 10 // max number of consecutive LBT hits allowed
#define SCAN_DWELL 10  // millis
#define SILENCE_DBM 80  // (presumed negative)
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
// there will be 4 of these: ACK RX/TX and Msg RX/TX
#define HASH_BUF_LEN 64

// Data buffers
uint8_t radioBuf[256];    // radio packet buffer
uint8_t radioDec[248];    // reedsolo decode buffer
uint8_t packetBuf[512];   // "a CVE waiting to happen"
uint16_t packetLen = 0;
int radioLen = 0;
int wantLen = 0;

// Msg hash ringbuffers
uint16_t rxAckBuf[HASH_BUF_LEN] = { 0 };
uint16_t txAckBuf[HASH_BUF_LEN] = { 0 };
uint16_t rxMsgBuf[HASH_BUF_LEN] = { 0 };
uint16_t txMsgBuf[HASH_BUF_LEN] = { 0 };
uint8_t rxAckPos = 0;
uint8_t txAckPos = 0;
uint8_t rxMsgPos = 0;
uint8_t txMsgPos = 0;

// Useful values from last sync packet
uint8_t dataFrags = 0; // Number of data packets in transmission
uint8_t wantFrags = 0; // Number of data packets left to receive
uint8_t lastIniTTL;  // for standardized output
uint8_t lastCurTTL;  // for standardized output

// Channel hopping related variables
bool scanning = true;
bool holdchan = false;  // if true, stay on this chan
bool inTXmode = false;
bool recvData = false;  // if true, we are on a data chan

bool softAFC = true;    // enable software AFC
uint16_t feiThre = 32;  // apply correction only if FEI outside this threshold
uint16_t currFei = 0;   // current FEI reading

// remember last 64 frequency error (FEI) readings
uint16_t feiHist[FEI_HIST_SIZE] = { 0 };
uint8_t feiHistPos = 0, feiHistLen = 0;

uint8_t currChan = 0;   // current channel NUMBER
uint8_t currDChIdx = 0; // current data chan INDEX in map
uint8_t currCChIdx = 0; // current ctrl chan INDEX in map

bool relaying = DFLT_RELAY;  // enable mesh relay function
bool noDeDup = false;  // disable duplicate filtering in RX
bool ezOutput = DFLT_EZOUTPUT;  // enable simple hexdump output of RX/TX events

esp_log_level_t logLevel = VERBOSITY;  // in case we want to query it

// Timing related variables
unsigned long pktStart = 0;   // millis when packet RX started (preamble detected)
unsigned long feiStart = 0;   // micros when FEI measurement started
unsigned long rssiStart = 0;  // millis when RSSI measurement started
unsigned long dataStart = 0;  // millis when data RX started
unsigned long lastHop = 0;    // millis when we last hopped ctrl chan
unsigned long chanTimer = 0;  // millis when we entered current channel
unsigned long lastRadioRx = 0; // millis when we last received (a valid packet)
unsigned long lastRadioTx = 0; // millis when we last transmitted

uint16_t txInertia = 0;   // INERTIA - millis to wait before TX
uint16_t txInerMAX = 600; // INERTIA - maximum value
unsigned long txBackOff = 0; // Tx retry backoff
uint16_t txBackMAX = 200;   // Tx max backoff
uint8_t lbtThreDBm = SILENCE_DBM;  // LBT threshold in -dBm (abs)

// REG_TEMP temperature cached last value
int8_t tempRegValue = 0;
unsigned long tempLastRead = 0;  // millis

// Specify frequency correction in integer Hz (not FSTEP nor PPM)
// This is explained elsewhere
int freqCorrHz = 0;  // static frequency correction in Hz
// obsolete
// int freqCorr = 0;  // static frequency correction in FSTEP units
                   // (~61Hz, same units as FEI, AFC etc)
// snapshot of current temperature at the time of most recent fcorr
int8_t fcorrRegTemp = 0;
int freqCoefHz = 0;  // thermal drift coefficient in Hz/degC

// baseline frequency calibration values
int8_t calibRegTemp = 0;  // snapshot of temperature at calibration time
int16_t calibFCorrHz = 0;  // frequency offset determined by last calbration

// RSSI, keep it simple
uint8_t pktRSSI = 0;  // absolute value; RSSI = -RssiValue/2[dBm]
uint8_t cChRSSI = 0;  // control chan RSSI (for LBT)
uint16_t sumDChRSSI = 0;  // sum of data channel RSSI, for averaging
uint8_t regRSSI_TS = 2;   // RSSI register time to settle [ms]
// actual formula: TS[ms] = 2 ^ (RssiSmoothing+1) / (4 * RX_BW[kHz])
// RssiSmoothing (0..7) = value of bits 0-2 of RegRssiConfig (0x0e)

uint8_t gtmTxPower = 0;  // set by gtmSetTxPower for easy access

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
  unsigned long tStamp;
  uint16_t msgLen;
  uint8_t msgObj[500];  // FIXME explain
};

struct ackDesc {
  uint8_t iniTTL;
  uint8_t curTTL;
  uint16_t hashID;
  unsigned long tStamp;
  uint8_t hops;
};

msgDesc msgQueue[MSG_QUEUE_SIZE];
ackDesc ackQueue[ACK_QUEUE_SIZE];

uint8_t msgQHead = 0, msgQTail = 0;
uint8_t ackQHead = 0, ackQTail = 0;

// number of calls to gtmLabLoop() per received packet
int pktLoops = 0;    // 20210303 PKTLOOPS

void echoCount(uint16_t hashID, bool isACK, bool isOwn = false);

bool processRxACK(uint16_t msgH16, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);
bool processRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);

// PACKET COUNTERS

// ACK packets received
// uint32_t cntRxPktACKTot = 0;  // total
uint32_t cntRxPktACKUni = 0;  // unique
// ACK packets transmitted
//uint32_t cntTxPktACKTot = 0;  // total
uint32_t cntTxPktACKRel = 0;  // relayed
// DATA objects received
uint32_t cntRxDataObjTot = 0;  // total
uint32_t cntRxDataObjUni = 0;  // unique
// DATA objects transmitted
uint32_t cntTxDataObjTot = 0;  // total
uint32_t cntTxDataObjRel = 0;  // relayed

// Raw packets received
uint32_t cntRxPktSYNC = 0;  // total SYNC
uint32_t cntRxPktDATA = 0;  // total DATA
uint32_t cntRxPktTIME = 0;  // total TIME
uint32_t cntRxPktACK = 0;   // total ACK

// Raw packets transmitted
uint32_t cntTxPktSYNC = 0;  // total SYNC
uint32_t cntTxPktDATA = 0;  // total DATA
uint32_t cntTxPktTIME = 0;  // total TIME
uint32_t cntTxPktACK = 0;   // total ACK

// ERROR COUNTERS
uint32_t cntErrPKTSTALL = 0;
uint32_t cntErrPRESTALL = 0;
uint32_t cntErrLOSTSYNC = 0;
uint32_t cntErrREEDSOLO = 0;
uint32_t cntErrCRC16BAD = 0;
uint32_t cntErrMISSCHAN = 0;

// LBT COUNTERS
uint32_t cntCChBusy = 0;
uint32_t cntCChFree = 0;
uint32_t cntDChBusy = 0;
uint32_t cntDChFree = 0;
uint32_t cntLBTHits = 0;  // counts consecutive LBT hits

uint32_t cntErrTXJAMMED = 0;

// Time tracking
timeTrack TTRK[TIMETRACK_PERIODS] = { 0 };
evCntTrack ETRK[TIMETRACK_PERIODS] = { 0 };
uint8_t ttCurPeriod = 0;

// user-definable event handler callbacks
// MSG rx handler
bool (* onRxMSG)(uint8_t *, uint16_t, uint8_t, uint8_t, uint8_t, uint16_t) = builtinRxMSG;
// ACK rx handler
bool (* onRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t) = builtinRxACK;
// Rx error handler
bool (* onRxERR)(uint16_t) = builtinRxERR;
// MSG Tx handler
bool (* onTxMSG)(uint16_t) = builtinTxMsgOK;
// ACK Tx handler
bool (* onTxACK)(uint16_t) = builtinTxAckOK;
// Temperature change handler
bool (* onTempChanged)(int8_t, int8_t) = builtinTempChg;


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
  uint8_t headPos = 5;

  if (mBuf[0] == MSG_CLASS_GROUP) {
    headPos = 12;   // destination address
  } else if (mBuf[0] == MSG_CLASS_P2P) {
    headPos = 15;   // destination + element 04
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


// Reset all counters
void gtmlabResetCounts()
{
  cntRxPktSYNC = cntRxPktDATA = cntRxPktACK = cntRxPktTIME = 0;
  cntTxPktSYNC = cntTxPktDATA = cntTxPktACK = cntTxPktTIME = 0;
  cntRxDataObjTot = cntRxDataObjUni = cntRxPktACK = cntRxPktACKUni = 0;
  cntTxDataObjTot = cntTxDataObjRel = cntTxPktACK = cntTxPktACKRel = 0;
  cntErrPRESTALL = cntErrPKTSTALL = cntErrLOSTSYNC = cntErrREEDSOLO = cntErrCRC16BAD = cntErrMISSCHAN = 0;
  cntCChFree = cntCChBusy = cntDChFree = cntDChBusy = cntErrTXJAMMED = 0;
}


// Get current period for time tracking
uint32_t ttGetPeriod()
{
  uint32_t tPeriod = int(millis()/TIMETRACK_MILLIS) % TIMETRACK_PERIODS;
  if (tPeriod != ttCurPeriod) {
    LOGI("TTRK period: %d", tPeriod);
    memset(&TTRK[tPeriod], 0, sizeof(timeTrack));
    ttCurPeriod = tPeriod;
  }
  return tPeriod;
}


unsigned long ttPeriodTime()
{
  ttGetPeriod();  // update to be sure
  return millis() - (int(millis()/TIMETRACK_MILLIS) * TIMETRACK_MILLIS);
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


// Look up a hash in MSG output queue
bool inMsgQueue(uint16_t hash16)
{
  for (int i = 0; i < MSG_QUEUE_SIZE; i++) {
    if (msgQueue[i].curTTL && (msgQueue[i].hashID == hash16))
      return true;
  }
  return false;
}


// Look up a hash in ACK output queue
bool inAckQueue(uint16_t hash16)
{
  for (int i = 0; i < ACK_QUEUE_SIZE; i++) {
    if (ackQueue[i].curTTL && (ackQueue[i].hashID == hash16))
      return true;
  }
  return false;
}

// Read radio temperature sensor
// The SX1276 manual specifies a strict seven-step method for reading
//  the temperature sensor, but this short version seems to work OK:
// "Put radio in STANDBY mode before polling the register"
int8_t getRadioTemp(uint16_t maxAgeSeconds)
{
  // only refresh value if it's older than max age requested
  if ((tempLastRead == 0) || ((tempLastRead + 1000 * maxAgeSeconds) < millis())) {
    uint8_t oldMode = LoRa.readRegister(REG_OP_MODE);
    // should always read 5, however...
    // oldMode = 1 (STDBY) may occur
    // oldMode = 4 (FSRX) noticed occasionally, even though we never set it
    //   (probably a transient value before mode=5 RX_CONTINUOUS)
    // Unsure whether we need to do anything special in these cases,
    //  for now we just log them to have a reference
    if (oldMode != MODE_RX_CONTINUOUS)
      LOGW("getRadioTemp MODE=%d", oldMode);
    // put radio in standby mode
    LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);
    // wait for mode change
    while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & IRQ1_MODEREADY));

    int8_t currTempRegV = -((int8_t)LoRa.readRegister(REG_TEMP));

    // return to old mode
    // FIXME this seems to cause instability (possibly rel to mode4 above)
    // LoRa.writeRegister(REG_OP_MODE, oldMode);
    // just return to RX CONT, which is our default mode
    LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

    // here we can hook an action to temperature changes
    if (currTempRegV != tempRegValue) {
      // invoke external handler
      // except on first (false) change from zero to current value
      if (tempLastRead) {
        unsigned long t0 = millis();  // track time spent in handler
        if ((* onTempChanged) != nullptr)
          onTempChanged(tempRegValue, currTempRegV);
        TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
      }
      // update values
      tempRegValue = currTempRegV;
    }
    tempLastRead = millis();
  }
  return(tempRegValue);
}


// Query current radio frequency by reading REG_FRF_* registers directly
// (never needed outside of desperate debug scenarios)
unsigned long getFrequency()
{
  unsigned long frf = 0;
  // officially we should read 24 bits, multiply by 32e6 and divide by 2^19
  // to avoid an integer capacity overrun, simplify this by treating 32 as 2^5,
  // and subtracting the left shift from the right shift
  frf += (1000000 * LoRa.readRegister(REG_FRF_MSB)) << 2;   // 2^16 * 2^5 / 2^19
  frf += (1000000 * LoRa.readRegister(REG_FRF_MID)) >> 6;   //  2^8 * 2^5 / 2^19
  frf += (1000000 * LoRa.readRegister(REG_FRF_LSB)) >> 14;  //  2^0 * 2^5 / 2^19

  return (frf);
}


// Set frequency, argument is channel number (0 to NUMCHANS)
void setChan(uint8_t chan)
{
  unsigned long tStart = millis();
  uint8_t readyBits = 0xd0;   // MODEREADY | RXREADY | PLLLOCK

  if (inTXmode)
    readyBits = 0xb0;   // MODEREADY | TXREADY | PLLLOCK

  // LoRa.setFrequency(curRegSet->baseFreq + (chan*curRegSet->chanStep));
  LoRa.setFrequency(curRegSet->baseFreq + (chan*curRegSet->chanStep) + freqCorrHz);  // quickfix

  bool locked = false;
  while(!locked) {
    uint8_t curr_IRQ1 = LoRa.readRegister(REG_IRQ_FLAGS_1);
    // MODEREADY | PLLLOCK = 0b10010000
    if ((curr_IRQ1 & readyBits) == readyBits)
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


// Tune to a control channel, argument is INDEX in ctrlChans[] map!
void setCtrlChanX(uint8_t cChanX)
{
  currCChIdx = cChanX;  // redundant?
  currCChIdx %= curRegSet->cChanNum;
  setChan(curRegSet->cChanMap[currCChIdx]);
}


// Tune to a control channel (next in sequence and increment)
// Call this repeatedly to traverse channel map in a scanning pattern
void setCtrlChan()
{
  currCChIdx++;
  currCChIdx %= curRegSet->cChanNum;
  setChan(curRegSet->cChanMap[currCChIdx]);
}


// Find a free channel to transmit
// Tune to one or more random channels, listen for a few millis (LBT_SILENCE), and if
//  RSSI is below silence floor (lbtThreDBm), return true - "clear to transmit"
// Will populate the global variables currDChIdx and currCChIdx with the indexes of
//   the tested channels, both in case of success or failure (this will allow the main
//   loop to pick up a potential incoming transmission without delay)
// WARNING: BLOCKING function will block for up to n * LBT_SILENCE millis
//
// Relevant: RSSISmoothing setting (default number of samples = 8 maybe too small?)
//
bool findClearChan(uint8_t needData)
{
  uint8_t tmpRSSI = 0;

  if (needData) {
    // Data Channels wanted, find this first
    // NOTE: Blocking section - can't do anything else while sniffing the data chan
    currDChIdx = random(curRegSet->dChanNum - 1);

    // We do not require the DATA chan straight away, because we haven't sent
    // our control (SYNC) packet yet. So, upon choosing a datachan sequence, we
    // check for a few things:
    // - check the exact datachan (to see is anything coming our way?) FIXME OVERKILL?
    // - check the next-in-seq datachan, where we will transmit (is anything there NOW?)
    // - check rest of datachans in sequence - mostly in case of persistent blockages
    for (int i = 0; i < (needData+1); i++) {
      setDataChanX((currDChIdx + i) % curRegSet->dChanNum);
      for (int j = 0; j < round(LBT_SILENCE/regRSSI_TS); j++) {
        delay(regRSSI_TS);  // ms
        tmpRSSI = LoRa.readRegister(REG_RSSI_VALUE);
        if (tmpRSSI < (lbtThreDBm<<1)) {
          LOGI("LBT DCh %02d(%02d) RSSI=-%d", (currDChIdx + i) % curRegSet->dChanNum,
              currChan, tmpRSSI>>1);
          cntDChBusy++;
          cntLBTHits++;
          // Before returning, hop back to original control chan to avoid a deadlock
          setCtrlChanX(currCChIdx);
          return false;
        }
      }
    }
    cntDChFree++;
  }

  // Ctrl Channel always needed, find it now
  // NOTE: this section is also blocking, although it doesn't technically NEED TO BE
  // (we sniff control channels all the time), but we do it to avoid complexity
  currCChIdx = random(curRegSet->cChanNum - 1);
  setCtrlChanX(currCChIdx);
  for (int j = 0; j < round(LBT_SILENCE/regRSSI_TS); j++) {
    delay(regRSSI_TS);  // ms
    tmpRSSI = LoRa.readRegister(REG_RSSI_VALUE);
    if (tmpRSSI < (lbtThreDBm<<1)) {
      LOGI("LBT CCh %02d(%02d) RSSI=-%d", currCChIdx, currChan, tmpRSSI>>1);
      cntCChBusy++;
      cntLBTHits++;
      return false;
    }
  }
  cntCChFree++;
  cntLBTHits = 0;
  return true;
}


// Find a free control channel, and an optional data channel sequence,
//  to transmit a packet of a given size (in bytes)
// Size = zero indicates a control packet only (like ACK)
bool findClearChansForDataSize(uint16_t needDataSize)
{
  // how many data channels? as many as the number of fragments!
  uint8_t nFrags = ceil(1.0 * needDataSize / TXFRAGSIZE);

  return findClearChan(nFrags);
}


// Reset receiver soft buffers and state variables
// Call this after a successful object reception or transmission, 
//   or to bail out of a receive operation that failed partway
// Should usually complete in <~1ms
void resetState(bool dirty)
{
  LOGD("--- RESET %s state", dirty ? "DIRTY" : "CLEAN");
  unsigned long ts = millis();
  wantFrags = 0;
  radioLen = 0;
  packetLen = 0;
  memset(radioBuf, 0, sizeof(radioBuf));
  memset(packetBuf, 0, sizeof(packetBuf));

  // if dirty, cycle through standby mode
  // this may also indirectly help with RSSI jamming issue
  if (dirty) {
    LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);
    while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80));
  }

  // expect long preamble
  LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xca); // 3 bytes

  // switch to RX mode in case we're not
  LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

  scanning = true;    // cycling through ctrl chans
  inTXmode = false;   // we're not in a transmission
  recvData = false;   // we're not receiving data
  pktStart = 0;   // we're not receiving a packet
  feiStart = 0;   // we're not measuring FEI
  currFei = 0;

  // reset TX inertia value
  // currently we use pure randomness, however this could be
  // refined to be a function of measured network congestion,
  // a timeslot-type arrangement etc
  txInertia = random(txInerMAX);
  LOGV("INERTIA=%dms", txInertia);

  // reset IRQ1 PREAMBLE AND SYNCWORD bits (by writing 1 to them)
  LoRa.writeRegister(REG_IRQ_FLAGS_1, 0x03);
  prev_IRQ1 = LoRa.readRegister(REG_IRQ_FLAGS_1);

  if (softAFC) {
    // reset frequency correction registers
    LoRa.writeRegister(REG_FSK_AFC_MSB, 0);
    LoRa.writeRegister(REG_FSK_AFC_LSB, 0);
  }

  // reset the radio's FIFO
  while (!((prev_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40))
    LoRa.readRegister(REG_FIFO);

  if (!holdchan)
    setCtrlChan();

    LOGV("IRQp: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" %s:%s:%s",
             BYTE_TO_BINARY(prev_IRQ1),
             BYTE_TO_BINARY(prev_IRQ2),
          (prev_IRQ1 & 0x02) ? "PR":"--",
          (prev_IRQ1 & 0x01) ? "SW":"--",
          (prev_IRQ2 & 0x04) ? "PL":"--");

  LOGV("rst state: %dms", millis()-ts);
}


// Check (from Arduino loop) if gtm main loop is in a busy state and
//  would prefer to not be held up by a lengthy non-gtm task
bool gtmlabBusy()
{
  // use existing state flags for now
  if ((scanning || holdchan) && !recvData && !inTXmode) {   // no operation in progress
    return false;
  }

  return true;
}


// Check (e.g from Arduino loop) if tx queue contains any objects
//   (which would be tx-ed on the next call to gtmTxTask)
bool gtmlabTxEmpty()
{
  if (msgQueue[msgQTail].curTTL > 0)
    return false;

  if (ackQueue[ackQTail].curTTL > 0)
    return false;

  return true;
}


void gtmSetTxPower(uint8_t txPower)
{
  uint8_t oldMode = LoRa.readRegister(REG_OP_MODE);
  // (required?) put radio in standby mode
  LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);

  LoRa.setTxPower(txPower);
  LoRa.writeRegister(REG_OP_MODE, oldMode);
  gtmTxPower = txPower;
}


// Configure the radio to the GTM waveform
// Will place AND LEAVE the radio in sleep mode
//
void setWaveform()
{
  // Set operating mode - sleep
  LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP);
  LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP); // the LoRa bit can only be written in sleep mode

  // Set bitrate divisor
  uint32_t bitrateReg = floor(1.0 * FXOSC / bitrate);
  uint8_t bitfracReg = floor(16 * (1.0 * FXOSC / bitrate - bitrateReg));

  // Set integer part
  LoRa.writeRegister(REG_BITRATE_MSB, (uint8_t) (bitrateReg >> 8));
  LoRa.writeRegister(REG_BITRATE_LSB, (uint8_t) (bitrateReg & 0xFF));
  // Set fractional part
  LoRa.writeRegister(REG_BITRATE_FRAC, bitfracReg & 0x0F);

  // Set frequency deviation
  uint32_t freqdevReg = round(1.0 * freqdev / FSTEP);

  LoRa.writeRegister(REG_FDEV_MSB, (uint8_t) (freqdevReg >> 8));
  LoRa.writeRegister(REG_FDEV_LSB, (uint8_t) (freqdevReg & 0xFF));

  // Set PA RAMP modulation shaping
  LoRa.writeRegister(REG_PA_RAMP, (uint8_t) 0x49);  // Gaussian BT=0.5
  
  // Set filter bandwidth to 25.0 kHz
  //   BwMant=01b, BwExp=4 -> 000 01 100
  LoRa.writeRegister(REG_RX_BW, RXBW_25000);

  // Set preamble detection
  LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xca); // 3 bytes preamble
  //LoRa.writeRegister(REG_PREAMBLE_DETECT, 0xaa); // 2 bytes preamble
  //LoRa.writeRegister(REG_PREAMBLE_DETECT, 0x8a); // 1 bytes preamble

  // Set RSSI smoothing (up to 32 samples (2dB accuracy) from default 8 (4dB))
  // TEMP DISABLED as usefulness is unclear; !ws0e04 to test
  //LoRa.writeRegister(REG_RSSI_CONFIG, 0x04);

  // Set Sync configuration - originally 0x51:
  // - AutoRestartRxMode = On, without wait for PLL lock
  // - Sync On, 2 bytes
  // NOTE: 0x91 (same as above, but restart WITH PLL lock)
  //   also works and may be a better option
  // LoRa.writeRegister(REG_SYNC_CONFIG, 0x51);
  LoRa.writeRegister(REG_SYNC_CONFIG, 0x91);

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
}


// run at startup to calibrate receiver
void rxCalibration()
{
  uint8_t oldImgCal = LoRa.readRegister(REG_IMAGE_CAL);
  LoRa.writeRegister(REG_IMAGE_CAL, oldImgCal | 0x40);  // bit 6 = ImageCalStart
  LoRa.setFrequency(curRegSet->baseFreq);
  while (LoRa.readRegister(REG_IMAGE_CAL) & 0x20);  // bit 5 = ImageCalRunning
}


// flick the receiver in and out of standby mode to clear a RSSI jam
void rxUnJam()
{
  uint8_t oldMode = LoRa.readRegister(REG_OP_MODE);
  // put radio in standby mode
  LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);
  // wait for mode change
  while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & IRQ1_MODEREADY));
  // return to old mode
  LoRa.writeRegister(REG_OP_MODE, oldMode);
}


// Apply temperature compensation if available and needed
void radioTempFreqComp() {
  // read radio temperature, gently
  int16_t currTemp = getRadioTemp(8);

  // If offset calibration was completed, and temperature has changed:
  // (NOTE: we check for a non-zero calibRegTemp as indication of
  //  calibration, since we don't usually calibrate at 0 deg Celsius)
  if (calibRegTemp && (currTemp != fcorrRegTemp)) {
    // Only apply temperature compensation if freqCoefHz was provided
    // Otherwise, leave freqCorrHz (and fCorrRegTemp) untouched, in
    //  case we want to evaluate freqCoefHz from the raw drift log
    if (freqCoefHz) {
      // Temperature-dependent frequency offset (drift) is added to
      //  the calibration offset (which is treated as a constant)
      int fTempOfsHz = freqCoefHz * (currTemp - calibRegTemp);
      freqCorrHz = calibFCorrHz + fTempOfsHz;
      fcorrRegTemp = currTemp;
      LOGI("dTemp=%d fComp=%d Hz", (currTemp - calibRegTemp), fTempOfsHz);
    }
  }
}


// Calculate mean of last nSamples FEI readings, excluding highest and lowest
//  values (trimmed mean), which are usually either misreadings or foreign
//  transmissions
// trim is an integer percent indicating depth of trimming relative to nSamples
// For example, trim=20 means bottom 10% and top 10% values will be discarded
//
// FEI input values in FSTEP units
// Return value in integer Hz
int feiTrimMeanHz(uint8_t nSamples, uint8_t trimPercent)
{
  int16_t tmpArr[FEI_HIST_SIZE];
  uint8_t arrLen = 0;
  int feiSum = 0;

  // load the required number of samples into tmpArr, newest first
  for (uint8_t i = 0; (i < feiHistLen) && (i < nSamples); i++) {
    tmpArr[i] = (int16_t) feiHist[(feiHistPos - i - 1 + FEI_HIST_SIZE) % FEI_HIST_SIZE];
    arrLen++;
  }

  // now bubblesort tmpArr
  for (uint8_t i = 1; i < arrLen; i++) {
    for (uint8_t j = i; (j > 0) && tmpArr[j-1] > tmpArr[j]; j--) {
      int16_t tmpval = tmpArr[j-1];
      tmpArr[j-1] = tmpArr[j];
      tmpArr[j] = tmpval;
    }
  }

  // we trim equally at both ends, so number is total amount / 2
  uint8_t trimNumber = round(trimPercent * arrLen / (2 * 100.0));

  // finally, calculate mean value excluding extremes
  for (uint8_t i = trimNumber; i < (arrLen - trimNumber); i++) {
    feiSum += tmpArr[i];
  }

  if (feiSum)
    return round((FSTEP * feiSum) / (arrLen - 2.0 * trimNumber));
  else
    return 0;
}


// Same as above, result in integer FSTEP units
int16_t feiTrimMean(uint8_t nSamples, uint8_t trimPercent)
{
  return round(feiTrimMeanHz(nSamples, trimPercent) / FSTEP);
}


// Call from Arduino setup() to initialize radio and data structures
void gtmlabInit()
{
#ifdef PIN_SCLK
  // radio sits on a non-default SPI
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
#endif

  LOGI("Board=%s, var=%s, id=%d. Region=%d", ARDUINO_BOARD, ARDUINO_VARIANT, BOARD_TYPE, ISM_REGION);

  // Configure the CS, reset, and IRQ pins
  LoRa.setPins(pinCS, pinRST, pinIRQ);

  if (!LoRa.begin(curRegSet->baseFreq)) {   // init radio at base frequency
    LOGE("Radio init failed. Check board type and pin definitions");
    while (true);
  }

  // Check radio version
  if (LoRa.readRegister(REG_VERSION) != 0x12) {
    LOGE("Unknown radio device");
    while (true);
  }

  rxCalibration();

  setWaveform();

  // By default, thermal AutoImageCalOn bit is set (0x82 | 10000010)
  //   do we want this disabled?
  //LoRa.writeRegister(REG_IMAGE_CAL, 0x02);

  // Set Tx power
  gtmSetTxPower(DFLT_POWER);  // defined in gtmConfig.h

  // Start RX mode
  LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);

  // Set frequency
  setCtrlChan();

  // Dump registers
  dumpRegisters(ESP_LOG_DEBUG);

  radioLen = 0;
  LOGI("Receiver up and running");
}


// Call from Arduino loop to perform RX and TX tasks
void gtmlabLoop()
{
  gtmlabRxTask();

  if (scanning && !recvData) {   // no operation in progress
    // Check and perform one TX task
    if (gtmlabTxTask()) {
      // something was transmitted
    }
  }
}


// called from main loop to check radio and perform RX tasks
// TIMING: <3ms when log=W and output disabled (due to serial bottleneck)
bool gtmlabRxTask()
{
  char rx_byte = 0;
  bool payReady = false;

  // Read IRQ registers once
  uint8_t curr_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2);
  uint8_t curr_IRQ1 = LoRa.readRegister(REG_IRQ_FLAGS_1);

  pktLoops++;    // 20210303 PKTLOOPS

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
      // PREAMBLE bit rising
      LOGD("PREAMBLE (t=0)");
      pktStart = millis();
      rssiStart = millis();  // start timer for RSSI reading
      pktRSSI = 0;
      pktLoops = 0;    // 20210303 PKTLOOPS
      scanning = false;
      if (!recvData) {
        // We are on a control (long preamble) channel, start FEI measurement
        feiStart = micros();  // warning, using micros rather than millis
        currFei = 0;
      }
    }

    if (((prev_IRQ1 & 0x03)!=0x03) && ((curr_IRQ1 & 0x03)==0x03)) {
      // SYNCWORD bit rising and PREAMBLE is up (or rising, see below)
      if (! pktStart) {
        // missed the preamble, but it's OK, start the timer now
        pktStart=millis();
        rssiStart = millis();  // start timer for RSSI reading
        pktRSSI = 0;
        pktLoops = 0;    // 20210303 PKTLOOPS
      }
      // if any FEI measurement in progress, stop it now - it's too late
      feiStart = 0;
      LOGD("PRE+SYNW (t=%d)", (millis()-pktStart));
      radioLen = 0;
      memset(radioBuf, 0, 256);
      scanning = false;   // Stay on channel
    }

    if (((prev_IRQ2 & 0x04)!=0x04) && ((curr_IRQ2 & 0x04)==0x04)) {
      // PAYREADY bit rising
      if (pktStart) {
        LOGD("PAYREADY (t=%d)", (millis()-pktStart));
        payReady = true;
      } else {
        LOGD("PAYREADY IGNORED");  // likely a misfire
        resetState(true);
        // FIXME - bailout?
      }
    }

    if (((prev_IRQ1 & 0x03)==0x03) && ((curr_IRQ1 & 0x03)!=0x03)) {
      // one of PREAMBLE/SYNCWORD bits falling ...
      if ((radioLen > 0) && !((curr_IRQ2 & 0x04)==0x04)) {
        // ... while PAYREADY bit is NOT up yet
        // lost sync while receiving packet - NOT GOOD
        // Read RSSI (debug purposes only)
        pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
        LOGI("LOSTSYNC %cCh=%02d pos %d/%d, RSSI=-%d t=%d", 
              holdchan ? '*' : (recvData ? 'D':'C'), 
              currChan, 
              radioLen-1, radioBuf[0], 
              pktRSSI>>1, (millis()-pktStart));
        cntErrLOSTSYNC++;

        // invoke external handler
        unsigned long t0 = millis();  // track time spent in handler
        if ((* onRxERR) != nullptr)
          onRxERR(RXERR_LOSTSYNC);
        TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

        // From practical observations, LOSTSYNC on a control chan
        //   usually indicates a bad "packet size" byte, which is 
        //   a hard one to recover from.
        // Abandon this packet and get ready for next one ASAP
        if ((!recvData) && radioBuf[0] > 16) {  // normal size=15
          resetState(true);
          return false;  // BAILOUT
        }
      }
    }

    // store values as "previous"
    prev_IRQ1=curr_IRQ1;
    prev_IRQ2=curr_IRQ2;
  }

  // Frequency corrections, part 1 (on control packets' long preamble)
  // The radio will measure the frequency error and load it into the FEI registers
  //  during preamble. The value becomes available "4 bit periods" from preamble
  //  detection, and is updated automatically
  // We measure and correct on the (long) preamble of the control packet of each
  //  received object, retain the setting for any subsequent data packets, and 
  //  reset the correction after the full object is received
  // The assumption is that the Tx frequency error is variable between nodes, but
  //  constant (in the short term) between packets from the same node
  //
  // FIXME: in production, this can and SHOULD be offloaded to the radio module
  //  (after all, this is what the A in AFC is about), but during experiments we
  //  prefer to be able to read and set it manually, for the cost of some CPU time
  //
  // Wait for 6 (more than 4) bit periods at 24kbps rate: 6 / 24000 = 0.25ms
  //  and also use a millis-based cutoff, for safety in case of micros rollover
  #define FEI_PERIOD_US 250    // wait for 250us from preamble detection...
  #define FEI_CUTOFF_MS 1      // ...or no more than 1 ms, which is a long time

  if (feiStart) {
    if (((micros() - feiStart) > FEI_PERIOD_US) || ((millis() - pktStart) > FEI_CUTOFF_MS)) {
      // FEI value must be ready by now
      currFei = (LoRa.readRegister(REG_FSK_FEI_MSB) << 8) | LoRa.readRegister(REG_FSK_FEI_LSB);
      feiStart = 0;  // done measuring
      if (softAFC && (abs((int16_t) currFei) > feiThre)) {
        LOGI("FEI=%d (%04x), AFC", (int16_t) currFei, currFei);
        LoRa.writeRegister(REG_FSK_AFC_MSB, (currFei >> 8) & 0xff);
        LoRa.writeRegister(REG_FSK_AFC_LSB, currFei & 0xff);
        rssiStart = millis();  // RESTART timer for RSSI reading
      } else {
        LOGD("FEI=%d (%04x)", (int16_t) currFei, currFei);
      }

      // record the value
      feiHist[feiHistPos] = currFei;
      feiHistPos = (feiHistPos + 1) % FEI_HIST_SIZE;
      if (feiHistLen < FEI_HIST_SIZE)
        feiHistLen++;
    }
  }
  // End of frequency corrections

  // If RSSI ready to read, read it now
  if (rssiStart && (millis() - rssiStart >= regRSSI_TS)) {
      pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
      rssiStart = 0;
      // Sometimes this returns a zero value, for no clear reason
      //   (possibly an AGC artifact?)
      // Log it for reference, but there's not really much to do about it
      if (!pktRSSI)
        LOGI("GOT RSSI=0!");
  }

  if(((curr_IRQ1 & 0x03)==0x03) && 
     ((curr_IRQ2 & 0x20) || !(curr_IRQ2 & 0x40))) {
    // if PREAMBLE|SYNCWORD are set, and (FIFOLEVEL or not FIFOEMPTY)

    if (curr_IRQ2 & 0x20) {  // FIFOLEVEL
      // FIFO above threshold, read FIFOTHRE bytes blindly
      for (int i=0; i<FIFOTHRE; i++) {
        radioBuf[radioLen++] = LoRa.readRegister(REG_FIFO);
      }
    }
    // Any more data left to read? Keep reading and find out!

    // FIFO now below threshold, read one-by-one until FIFOEMPTY
    while (!((curr_IRQ2 = LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40)) {
      radioBuf[radioLen++] = LoRa.readRegister(REG_FIFO);

      // keep watch for PAYREADY signaling at all times
      if (curr_IRQ2 & 0x04) {
        payReady = true;
      }
    }    
    prev_IRQ2=curr_IRQ2;

    // check PAYLOAD_READY IRQ2 bit (read and saved above)
    if (payReady) {
      if (rssiStart) {
        // If waiting for RSSI, read it ASAP
        // FIXME check docs, it may be too late to read RSSI here
        pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
        rssiStart = 0;
      }

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
          TTRK[ttGetPeriod()].timeRX += (millis() - pktStart);
          rxPacket(radioDec, radioLen-8, pktRSSI>>1);

          // not really needed, but helps with debugging
          memset(radioBuf, 0, sizeof(radioBuf));

        } else if (radioLen != (radioBuf[0]+1)) {
          // Reed-Solomon can't fix a packet of incorrect length, but may be able
          //  to fix a packet with CORRECT length and 1-2 corrupt bytes at the end
          // So, if the received length is a little less than expected (based on
          //  byte 0 of a GTM packet), we can try to lie about the length (this
          //  will pad the packet with garbage) and hope reedsolo fixes it for us

          LOGW("FIX BUF SIZE: %d->%d", radioLen, radioBuf[0]+1);
          radioLen = radioBuf[0]+1;
          rs.begin(radioLen, 1);
          if (rs.Decode(radioBuf, radioDec) == 0) {
            LOGI("REEDSOLO-FIX!");  // GREAT SUCCESS!
            // Packet OK, send it for further processing
            TTRK[ttGetPeriod()].timeRX += (millis() - pktStart);
            rxPacket(radioDec, radioLen-8, pktRSSI>>1);
            // not really needed, but helps with debugging
            memset(radioBuf, 0, sizeof(radioBuf));
          }

        } else {
          LOGI("REEDSOLO FAIL %cCh=%02d, len=%d, RSSI=-%d", 
                holdchan ? '*' : (recvData ? 'D':'C'), 
                currChan, radioLen, pktRSSI>>1);
          cntErrREEDSOLO++;

          // invoke external handler
          unsigned long t0 = millis();  // track time spent in handler
          if ((* onRxERR) != nullptr)
            onRxERR(RXERR_REEDSOLO);
          TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

          resetState(true);
        }
      } else {
        LOGD("Invalid rxLen");
        resetState(true);
      }

      // Finished processing a radio packet, clean up a bit

      // Frequency corrections, part 2 (post SYNC packet)
      if (wantFrags) {
        // If more packets from the current transmission are incoming:

        // The currFei variable contains the FEI of the transmitter
        //  as measured during SYNC packet preamble, and loaded into
        //  the AFC registers **at that time** if [conditions are met]
        //  (see "frequency corrections, part 1" above)
        // Now: since all remaining packets of the current transmission
        //  come from the same physical radio, the same freq. correction
        //  should be retained and used for all DATA packets in sequence
        // HOWEVER:
        // The AFC registers sometimes get cleared after each packet
        //  (is this documented?), which can cause the DATA packets to
        //  be received out-of-tune (usualy resulting in a failure)

        // Here we REload currFei into AFC registers, if required

        if (softAFC && (abs((int16_t) currFei) > feiThre)) {
          LoRa.writeRegister(REG_FSK_AFC_MSB, (currFei >> 8) & 0xff);
          LoRa.writeRegister(REG_FSK_AFC_LSB, currFei & 0xff);
        }
      }

      pktStart = 0;  // no packet being received right now
      feiStart = 0;
      radioLen = 0;
      return true;
    }  // if payReady
  }

  // in recvData mode, we just jumped on a data chan and are 
  //   currently waiting for a preamble?
  // if we missed the preamble, make sure we don't wait forever
  //if (recvData && (! (curr_IRQ1 & 0x02)) && 
  if (recvData && (! pktStart) && 
      (millis() - chanTimer > PRE_TIMEOUT)) {
    pktRSSI = LoRa.readRegister(REG_RSSI_VALUE);
    LOGI("PRESTALL %cCh=%02d, RSSI=-%d (t=%d)",
          holdchan ? '*' : (recvData ? 'D':'C'), 
          currChan,
          pktRSSI>>1, millis()-chanTimer);
    cntErrPRESTALL++;

    // invoke external handler
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxERR) != nullptr)
      onRxERR(RXERR_PRESTALL);
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

    resetState(true);   // BAILOUT
    return false;
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
    cntErrPKTSTALL++;

    // invoke external handler
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxERR) != nullptr)
      onRxERR(RXERR_PKTSTALL);
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

    resetState(true);  // dirty
    return false;
  }

  if (scanning && !recvData) {   // no operation in progress
    // Check and apply temperature compensations
    radioTempFreqComp();

    // if scanning, HOP to next ctrl chan every SCAN_DWELL millis
    if (((millis()-lastHop) > SCAN_DWELL) && !holdchan) {
      setCtrlChan();
      lastHop = millis();
    }
  }

  return true;
}


// called from main loop to check for TX tasks and execute the first in queue
// TIMING: <230ms when log=W and output disabled (due to serial bottleneck)
bool gtmlabTxTask()
{
    bool txHold = false;  // hold the transmission of current object (INERTIA)
    bool txLocal = true;  // the message is originated locally
    int txRes = -1;  // TX Result

    if (holdchan) {
      // can't TX while holding chan
      return false;
    }

    if (txBackOff > millis())
      txHold = true;    // Hold TX if backoff engaged

    unsigned long tStart = millis();

    // Quick notes on INERTIA feature:
    // When relaying a received object, all nodes on the network would normally
    // start the relay process at the same time, increasing the risk of collisions
    // such as reported in issue #3
    // The INERTIA feature mitigates this by introducing a random delay before
    // the transmission of a relayed object (but not locally originated objects,
    // which do not suffer from this issue)
    // It is similar, but not identical to the concept of random backoff; the
    // difference is the delay here is applied proactively not reactively

    // Anything queued for sending?
    if (msgQueue[msgQTail].curTTL > 0) {
      if (msgQueue[msgQTail].iniTTL != msgQueue[msgQTail].curTTL) {
        // curTTL == iniTTL indicates a locally originated object (not relayed)
        // only for relayed objects, use INERTIA to delay TX start
        txLocal = false;
        if (millis() - msgQueue[msgQTail].tStamp < txInertia) {
          txHold = true;
        }
      }

      if (! txHold) {
        if (findClearChansForDataSize(msgQueue[msgQTail].msgLen)) {
          TTRK[ttGetPeriod()].timeLBT += (millis() - tStart);
          LOGI("LBT(%dms) OK, can TX %s MSG(pos:%d)", (millis() - tStart),
                txLocal ? "OWN":"RLY", msgQTail);
          tStart = millis();

          // Transmitter ON
          txStart();
          txRes = txSendMsg(msgQueue[msgQTail].msgObj, msgQueue[msgQTail].msgLen,
                            msgQueue[msgQTail].iniTTL, msgQueue[msgQTail].curTTL);
          TTRK[ttGetPeriod()].timeTX += (millis() - tStart);

          // record the time
          lastRadioTx = millis();

          // advance the queue
          memset(&(msgQueue[msgQTail]), 0, sizeof(struct msgDesc));
          msgQTail = (msgQTail+1) % MSG_QUEUE_SIZE;
          resetState(false);
          // Transmitter OFF

          if(txLocal && (txRes > 0)) {
            echoCount(txRes, false, true);  // count an own MSG
            unsigned long t0 = millis();  // track time spent in handler
            if (! onTxMSG(txRes)) {
              LOGD("onTxMSG handler failed");
            }
            TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
          }
          return true;
        } else {
          TTRK[ttGetPeriod()].timeLBT += (millis() - tStart);
          int txBack = random(txBackMAX);
          txBackOff = millis() + txBack;
          LOGI("LBT(%dms) SKIPPED TX MSG(pos:%d) backoff=%d", (millis() - tStart),
                msgQTail, txBack);
        }
      }

    } else if (ackQueue[ackQTail].curTTL > 0) {
      if (ackQueue[ackQTail].iniTTL != ackQueue[ackQTail].curTTL) {
        // curTTL == iniTTL indicates a locally originated object (not relayed)
        txLocal = false;
        if (millis() - ackQueue[ackQTail].tStamp < txInertia) {
          txHold = true;
        }
      }
      if (! txHold) {
        if (findClearChan(0)) {  // dataChans = 0 = none required
          TTRK[ttGetPeriod()].timeLBT += (millis() - tStart);
          LOGI("LBT(%dms) OK, can TX %s ACK(pos:%d)", (millis() - tStart),
                txLocal ? "OWN":"RLY", ackQTail);
          tStart = millis();

          // Transmitter ON
          txStart();
          txSendAck(ackQueue[ackQTail].hashID, ackQueue[ackQTail].hops,
                    ackQueue[ackQTail].iniTTL, ackQueue[ackQTail].curTTL);
          TTRK[ttGetPeriod()].timeTX += (millis() - tStart);

          // record the time
          lastRadioTx = millis();

          // advance the queue
          uint16_t tHash = ackQueue[ackQTail].hashID;
          memset(&(ackQueue[ackQTail]), 0, sizeof(struct ackDesc));
          ackQTail = (ackQTail+1) % ACK_QUEUE_SIZE;
          resetState(false);
          // Transmitter OFF

          if(txLocal) {
            echoCount(tHash, true, true);  // count an own ACK
            unsigned long t0 = millis();  // track time spent in handler
            if (! onTxACK(tHash)) {
              LOGD("onTxACK handler failed");
            }
            TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
          }
          return true;
        } else {
          TTRK[ttGetPeriod()].timeLBT += (millis() - tStart);
          int txBack = random(txBackMAX);
          txBackOff = millis() + txBack;
          LOGI("LBT(%dms) SKIPPED TX ACK(pos:%d) backoff=%d", (millis() - tStart),
                ackQTail, txBack);
        }
      }
    }

    // Excessive number of consecutive LBT hits?
    if (cntLBTHits > LBT_MAX_HITS) {
      cntErrTXJAMMED++;
      LOGW("LBT HITS: %d, reset RX", cntLBTHits);
      cntLBTHits = 0;
      // Our RSSI detector may be jammed, try to un-jam it
      // Flicking the radio to standby and back usually fixes the problem
      rxUnJam();
    }
    return false;  // nothing was sent
}


// called from radio receiver for each good packet received
//   (uRSSI = unsigned RSSI)
int rxPacket(uint8_t * rxBuf, uint8_t rxLen, uint8_t uRSSI)
{
  uint16_t msgH16;
  bool isUnique = true;  // first time we see this object

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
    cntErrCRC16BAD++;

    // invoke external handler
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxERR) != nullptr)
      onRxERR(RXERR_CRC16BAD);
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

    return 0;
  }

  // good packet; drop the CRC
  rxLen-=2;

  // record the time
  lastRadioRx = millis();

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
    cntRxPktSYNC++;

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
    cntRxPktTIME++;

  } else if (rxBuf[1] == PKT_TYPE_ACK) {
    // ACK packet, indicates the successful delivery of a P2P message
    msgH16 = (rxBuf[2]<<8)+rxBuf[3];
    LOGI("%s ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d",
      chanDesc, msgH16, 
      (rxBuf[4]>>4), (rxBuf[4] & 0x0f), rxBuf[5]);
    cntRxPktACK++;

    // pass the ACK object to routing/handling layer
    processRxACK(msgH16, (rxBuf[4]>>4), (rxBuf[4] & 0x0f), rxBuf[5], uRSSI, currFei);
    resetState();

  } else if (rxBuf[1] == PKT_TYPE_DATA) {
    // DATA packet, a fragment of a protocol message of random length
    LOGI("%s DATA(2): len=%d, fragIDX=%d", chanDesc, rxBuf[2], rxBuf[3]);
    cntRxPktDATA++;

    if (!recvData) {
      // TROUBLE! received DATA packet on CTRL channel, this is never good
      LOGW("MISSCHAN, f=%d", getFrequency());
      cntErrMISSCHAN++;

      // invoke external handler
      unsigned long t0 = millis();  // track time spent in handler
      if ((* onRxERR) != nullptr)
        onRxERR(RXERR_MISSCHAN);
      TTRK[ttGetPeriod()].timeEvt += (millis() - t0);

      resetState(true);
      return 0;
    }

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
      // no more frags; submit the assembled object and hop to control channel
      cntRxDataObjTot++;

      processRxMSG(packetBuf, packetLen, lastIniTTL, lastCurTTL, int(sumDChRSSI/(double)dataFrags), currFei);
      packetLen=0;
      // will reset buffers, hop to cch and start scanning
      resetState();
    }

  } else {
    LOGI("%s UNK (%d)", chanDesc, rxBuf[1]);    
  }
  return 0;
}


// simple builtin MSG handler - hexlified output
// (uRSSI because the RSSI is unsigned)
bool builtinRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  if (ezOutput) {  // easy output enabled
    // output in standardized format (inittl, curttl, uRSSI, | ,hexmsg)
    printf("RX_MSG:");
    printf("%02x%02x%02x|", iniTTL, curTTL, uRSSI);
    for (int i=0; i<mLen; i++)
      printf("%02x", mBuf[i]);
    printf("\n");
  }
  return true;
}


// simple builtin ACK handler - hexlified output
bool builtinRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("RX_ACK:%04x%02x%02x%02x\n", hashID, ((hops & 0x0f)<<4) | (iniTTL & 0x0f), curTTL, uRSSI);
  return true;  
}


// simple builtin RX Error handler
bool builtinRxERR(uint16_t error)
{
  // do nothing
  return true;
}


// simple builtin TX MSG succcess handler - output is hex HashID of message
bool builtinTxMsgOK(uint16_t hashID)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("TX_MSG:%04x\n", hashID);
  return true;
}


// simple builtin TX ACK succcess handler - output is hex HashID
bool builtinTxAckOK(uint16_t hashID)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("TX_ACK:%04x\n", hashID);
  return true;
}


// simple builtin Temperature change handler
bool builtinTempChg(int8_t tempOld, int8_t tempNew)
{
  LOGI("TEMP: %d->%d", tempOld, tempNew);
  // do nothing
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
//
// FIXME-TERMS in this case, for clarity we may prefer to refer to it as
//  a FRAME instead of a PACKET
//
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

  return 0;
}


// Takes a packet payload,
// prepends packet header, appends crc16 and reed-solomon code
// and then invokes txPacket() to SEND the packet over radio
//
// FIXME-TERMS: takes a PACKET and appends a FRAME header/footer
//
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

  LOGV("AIR_SIZE=%d", pktLen + 8);

  // and finally transmit outbuf
  return txPacket(radioBuf, pktLen + 8, isCtrl);
}


// Send a SYNC packet (will set channel to currCChIdx; expects TX mode on)
int txSendSync(uint8_t chIDX, uint8_t frags, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = chIDX;
  mBuf[1] = frags;
  mBuf[2] = iniTTL;
  mBuf[3] = curTTL;
  
  // currCChIdx is selected in findClearChan(), which should be called first
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d SYNC(1): chIDX=%d, frags=%d, iniTTL=%d, curTTL=%d", 
        currChan, chIDX, frags, iniTTL, curTTL);
  int r = txEncodeAndSend(mBuf, 4, PKT_TYPE_SYNC);
  cntTxPktSYNC++;

  return r;
}


// Send an ACK packet (will set channel to currCChIdx; expects TX mode on)
int txSendAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  uint8_t mBuf[4];
  mBuf[0] = (hashID >> 8);
  mBuf[1] = (hashID & 0xff);
  mBuf[2] = (hops & 0x0f)<<4;
  mBuf[2] |= (iniTTL & 0x0f);
  mBuf[3] = curTTL;

  // currCChIdx is selected in findClearChan(), which should be called first
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d ACK (3): hash=0x%04x, hops=%d, iniTTL=%d, curTTL=%d", 
        currChan, hashID, hops, iniTTL, curTTL);
  int r = txEncodeAndSend(mBuf, 4, PKT_TYPE_ACK);
  cntTxPktACK++;

  // save to ringbuffer
  txAckBuf[txAckPos++] = hashID;
  txAckPos %= HASH_BUF_LEN;

  return r;
}


// Send a TIME packet (will set channel to currCChIdx; expects TX mode on)
int txSendTime(uint64_t time32)
{
  uint8_t mBuf[4];
  // timestamp
  mBuf[0] = (time32 >> 24) & 0xff;
  mBuf[1] = (time32 >> 16) & 0xff;
  mBuf[2] = (time32 >> 8) & 0xff;
  mBuf[3] = time32 & 0xff;

  // currCChIdx is selected in findClearChan(), which should be called first
  setChan(curRegSet->cChanMap[currCChIdx]); 

  LOGI("TX CCh=%02d TIME(0): 0x%08x", currChan, time32);
  int r = txEncodeAndSend(mBuf, 4, PKT_TYPE_TIME);
  cntTxPktTIME++;

  return r;
}



// Send a message
// This is a complex action involving several packets
// - will set channel using the values provided in currCChIdx and currDChIdx
// - expects TX mode on, and will leave it unchanged
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

  // send sync packet
  txSendSync(currDChIdx, nFrags, iniTTL, curTTL);

  // allow some time for receivers to switch channel
  // from experiments, a value of 10ms works well here
  // delay(txSyncDelay - txPackDelay);
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
    setDataChanX(currDChIdx);  // will block until settled

    // send the fragment
    txEncodeAndSend(oBuf, oLen, PKT_TYPE_DATA);
    cntTxPktDATA++;

    // output after data upload, to avoid any uncontrolled delays
    LOGI("TX DCh=%02d DATA(2): len=%d, fragIDX=%d", currChan, oLen-2, i);
    while (!LoRa.readRegister(REG_IRQ_FLAGS_2) & 0x40);  // wait for FifoEmpty

    delay(txPackDelay);  // wait for others
  }
  cntTxDataObjTot++;
  return msgH16;
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
  msgQueue[msgQHead].tStamp = millis();
  msgQueue[msgQHead].msgLen = msgLen;
  memcpy(msgQueue[msgQHead].msgObj, msgObj, msgLen);
  msgQHead = (msgQHead+1) % MSG_QUEUE_SIZE;

  return true;
}


// Queue an ACK for sending (when channel is clear)
bool txEnQueueACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  if (ackQueue[ackQHead].curTTL>0) {
    // buffer is full, retry later
    return false;
  }
  ackQueue[ackQHead].iniTTL = iniTTL;
  ackQueue[ackQHead].curTTL = curTTL;
  ackQueue[ackQHead].tStamp = millis();
  ackQueue[ackQHead].hashID = hashID;
  ackQueue[ackQHead].hops = hops;
  ackQHead = (ackQHead+1) % ACK_QUEUE_SIZE;

  return true;
}


// called from rxPacket to submit an incoming ACK object for processing
bool processRxACK(uint16_t msgH16, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  bool isUnique = true;  // first time we see this object

  echoCount(msgH16, true);  // count an ACK

  if (inRingBuf(msgH16, rxAckBuf)) {
    isUnique = false;
    LOGI("ACK SEEN BEFORE");
  } else {
    cntRxPktACKUni++;
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

    // Is this an echo of an ACK that we sent?
    if (inRingBuf(msgH16, txAckBuf)) {
      LOGI("ACK SENT BEFORE");
    // have we already queued a copy for sending?
    } else if (inAckQueue(msgH16)) {
      LOGI("ACK IN TX QUEUE");
    } else {
      // if first seen, and curTTL>1, it's a relayable ACK
      if (curTTL > 1) {
        ////////// RELAYING //////////
        LOGI("RELAYACK (%s)", (relaying ? "ON!":"OFF"));
        if (relaying) {
          // decrement curTTL and enqueue for TX
          // FIXME catch error
          txEnQueueACK(msgH16, hops, iniTTL, (curTTL - 1));
          cntTxPktACKRel++;
        }
      }
    }
  }

  if (isUnique || noDeDup) {
    // new ACK received - call external handler if defined
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxACK) != nullptr) {
      if (! onRxACK(msgH16, hops, iniTTL, curTTL, uRSSI, FEI)) {
        LOGD("onRxACK handler failed");
      }
    }
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
  }

  return true;
}


// called from rxPacket to submit an incoming MSG object for processing
bool processRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  bool isUnique = true;  // first time we see this object
  uint16_t msgH16 = msgHash16(mBuf);

  LOGI("RX complete: len=%d, hash=0x%04x, time=%dms, RSSI=-%d",
       mLen, msgH16, (millis()-dataStart), uRSSI);
  echoCount(msgH16, false);  // count an MSG

  if (inRingBuf(msgH16, rxMsgBuf)) {
    LOGI("MSG SEEN BEFORE");
    isUnique = false;
  } else {
    cntRxDataObjUni++;
    rxMsgBuf[rxMsgPos++] = msgH16;
    rxMsgPos %= HASH_BUF_LEN;

    // Is this an echo of a message that we sent?
    if (inRingBuf(msgH16, txMsgBuf)) {
      LOGI("MSG SENT BEFORE");
    // have we already queued a copy for sending?
    } else if (inMsgQueue(msgH16)) {
      LOGI("MSG IN TX QUEUE");
    } else {
      // if first seen, and curTTL>1, it's a relayable message
      if (curTTL>1) {
        ////////// RELAYING //////////
        LOGI("RELAYMSG (%s)", (relaying ? "ON!":"OFF"));
        if (relaying) {
          // decrement curTTL and enqueue for TX
          // FIXME catch error
          txEnQueueMSG(mBuf, mLen, iniTTL, (curTTL - 1));
          cntTxDataObjRel++;
        }
      }
    }
  }

  if (isUnique || noDeDup) {
    // New message received - call external handler if defined
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxMSG) != nullptr) {
      if (! onRxMSG(mBuf, mLen, iniTTL, curTTL, uRSSI, FEI)) {
        LOGD("onRxMSG handler failed");
      }
    }
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
  }

  return true;
}



////////// Some additional tools & experiments below //////////

// ECHO COUNTER
// For each object TXd or RXd, count the number of copies we received back
// Two important applications:
// - roughly estimate the size of the reachable network
// - puddingproof the relaying efficiency of our nodes (when starting up
//   a new LAB relay, do we CONSISTENTLY see an extra echo? compare with
//   GTM relays)
// Call echoCount(hashID, isACK), the number of echoes will be logged at INFO

#define ECHOSIMTRACK 8  // max echoes to track simultaneously, of each type
struct echoDesc {
  unsigned long tStamp;
  uint16_t hashID;
  uint8_t count;
};

// one dataset for MSGs, one for ACKs
echoDesc Echoes[ECHOSIMTRACK * 2] = {0};
uint8_t echoHead[2] = {0};

// make some of these variables externally available
uint8_t echoCur = 0;
uint16_t echoAvg = 0;  // x100
char echoFmt[33] = {0};  // formatted


void echoCount(uint16_t hashID, bool isACK, bool isOwn)
{
  uint8_t hx = 0;  // which dataset, MSG or ACK
  uint8_t i = 0;   // index into arrays

  if (isACK)
    hx = 1;

  echoDesc * e = & Echoes[hx * ECHOSIMTRACK];

  bool found = false;
  for (i = 0; i < ECHOSIMTRACK; i++) {
    if (e[i].hashID == hashID) {
      found = true;
      break;
    }
  }
  if (!found) {
    i = echoHead[hx];
    e[i].hashID = hashID;
    e[i].count = 0;
    e[i].tStamp = millis();
    echoHead[hx] = (echoHead[hx] + 1) % ECHOSIMTRACK;
  }

  if (!isOwn) {
    // only increment for non-own packets
    e[i].count++;
  }

  int cnt = 0;
  int sum = 0;

  // Format the last ECHOSIMTRACK message echo counts, new-to-old
  // FIXME? is the reversed arrow-of-time a problem?
  char formatted[33] = {0};
  int pos = 0;
  for (int j = 0; j < ECHOSIMTRACK; j++) {
    // sum for average
    if ((i != j) && (e[j].count >0)) {
      sum += e[j].count;
      cnt ++;
    }

    // format for output
    pos += sprintf(formatted + pos, "%d ", e[(ECHOSIMTRACK+i-j) % ECHOSIMTRACK].count);
  }
  if (cnt) {
    if (! isACK) {
      // copy to extern variables
      echoAvg = 100 * sum / cnt;
      echoCur = e[i].count;
      strncpy(echoFmt, formatted, 32);
    }
    LOGI("ECHO(%s:%04x)=%d t=%d AVG=%02.2f [ %s]", isACK ? "ACK":"MSG", hashID,
         e[i].count, millis() - e[i].tStamp, 1.0 * sum / cnt, formatted);
  }
  return;
}

////
