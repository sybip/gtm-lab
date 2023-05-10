//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#ifndef GTMRADIO_H
#define GTMRADIO_H

#include <Arduino.h>
#include "logging.h"

// BOARD TYPE SETTING
#ifndef BOARD_TYPE
// We can autodetect board type from ARDUINO_* compile flag

#ifdef ARDUINO_TBeam
#define BOARD_TYPE 1

#elif ARDUINO_ESP32_DEV
#define BOARD_TYPE 2

#elif ARDUINO_ESP32_MICROMOD
#define BOARD_TYPE 3

#else
#error "Unrecognized board type, try setting BOARD_TYPE in gtmConfig.h"

#endif
#endif

// Radio packet types
#define PKT_TYPE_TIME 0
#define PKT_TYPE_SYNC 1
#define PKT_TYPE_DATA 2
#define PKT_TYPE_ACK  3

// Different RX Error types that we track separately
#define RXERR_DEFAULT 0
#define RXERR_PRESTALL 1
#define RXERR_PKTSTALL 2
#define RXERR_LOSTSYNC 3
#define RXERR_REEDSOLO 4
#define RXERR_CRC16BAD 5
#define RXERR_MISSCHAN 6

// constants specific to SX127x radio
#define FXOSC 32E6  // freq of xtal oscillator in Hz = 32MHz
#define FSTEP 61.03515625  // synth freq step in Hz = (FXOSC/2^19)

// regional radio frequency and channel settings
struct regSet {
  uint32_t baseFreq;      // base frequency
  uint32_t chanStep;      // channel step
  uint8_t tChanNum;       // total number of chans: data + ctrl + unused (if any)
  uint8_t cChanNum;       // number of control channels
  uint8_t cChanMap[8];    // map of control channels
  uint8_t dChanNum;       // number of data channels
  uint8_t dChanMap[64];   // map of data channels
};


// state variables, from gtmRadio.cpp
extern bool scanning;
extern bool holdchan;       // if true, stay on this chan
extern bool inTXmode;
extern bool recvData;       // if true, we are on a data chan

extern int freqCorrHz;        // static freq correction in Hz
// extern int freqCorr;        // OBSOLETE static freq correction (in FSTEP units!)
extern int8_t fcorrRegTemp; // snapshot of temperature at the time of fcorr
extern int freqCoefHz;      // empirical thermal drift coefficient in Hz/degC
extern int8_t calibRegTemp;  // snapshot of temperature at calibration time
extern int16_t calibFCorrHz;  // frequency offset determined by last calbration

extern bool softAFC;        // enable software AFC
extern uint16_t feiThre;    // software AFC threshold
extern uint8_t currChan;    // current channel NUMBER
extern uint8_t currDChIdx;  // current data chan INDEX in map
extern uint8_t currCChIdx;  // current ctrl chan INDEX in map
extern regSet regSets[];
extern regSet * curRegSet;
extern uint8_t txSyncDelay;   // millis to wait between sync packet and first data packet
extern uint8_t txPackDelay;   // millis to wait between data packets
extern esp_log_level_t logLevel;
extern unsigned long chanTimer;
extern uint8_t lbtThreDBm;    // LBT threshold in -dBm (abs)
extern uint8_t gtmTxPower;    // Current Tx Power
extern unsigned long lastRadioRx; // millis when we last received (a valid packet)
extern unsigned long lastRadioTx; // millis when we last transmitted

// COUNTERS

// Raw packets received
extern uint32_t cntRxPktSYNC;  // total SYNC
extern uint32_t cntRxPktDATA;  // total DATA
extern uint32_t cntRxPktTIME;  // total TIME
extern uint32_t cntRxPktACK;   // total ACK
// Raw packets transmitted
extern uint32_t cntTxPktSYNC;  // total SYNC
extern uint32_t cntTxPktDATA;  // total DATA
extern uint32_t cntTxPktTIME;  // total TIME
extern uint32_t cntTxPktACK;   // total ACK

// ERROR COUNTERS
extern uint32_t cntErrPKTSTALL;
extern uint32_t cntErrPRESTALL;
extern uint32_t cntErrLOSTSYNC;
extern uint32_t cntErrREEDSOLO;
extern uint32_t cntErrCRC16BAD;
extern uint32_t cntErrMISSCHAN;
extern uint32_t cntErrTXJAMMED;

// LBT COUNTERS
extern uint32_t cntCChBusy;
extern uint32_t cntCChFree;
extern uint32_t cntDChBusy;
extern uint32_t cntDChFree;

// TIME TRACKING
#define TIMETRACK_MILLIS (3600 * 1000)   // one hour per period
#define TIMETRACK_PERIODS 25     // total 25 periods

// Time tracker
struct timeTrack {
  uint32_t timeTX;    // time spent in TX
  uint32_t timeRX;    // time spent in RX
  uint32_t timeLBT;   // time spent listening before TX
  uint32_t timeEvt;   // time spent in handler events
};

extern timeTrack TTRK[TIMETRACK_PERIODS];
extern uint8_t ttCurPeriod;
uint32_t ttGetPeriod();
unsigned long ttPeriodTime();

// event (mostly error) count tracker
struct evCntTrack {
  uint16_t errors;    // total errors
};

extern evCntTrack ETRK[TIMETRACK_PERIODS];

// FEI history
#define FEI_HIST_SIZE 64
extern uint16_t feiHist[FEI_HIST_SIZE];
extern uint8_t feiHistPos, feiHistLen;

// event handler functions
extern bool (* onTempChange)(int8_t, int8_t);

bool builtinTempChg(int8_t tempOld, int8_t tempNew);

// additive CRC16 function
uint16_t CRC16_add(uint8_t b, uint16_t crc = 0);

// Dump all radio registers, in hex and binary (datasheet for details)
// Argument: ESP log level, -1 for printf
void dumpRegisters(int logLevel);

// Reset all counters
void gtmlabResetCounts();

// setTxPower (in dBm)- wrapper for LoRa.setTxPower()
// use this instead of calling setTxPower() directly
void gtmSetTxPower(uint8_t txPower);

// Look up a hash in one of the hash ringbuffers
bool inRingBuf(uint16_t needle, uint16_t *haystack);

// Read radio temperature sensor (uncalibrated but still useful)
int8_t getRadioTemp(uint16_t maxAgeSeconds = 0);

// Query current radio frequency by reading REG_FRF_* registers directly
unsigned long getFrequency();

// Mean value of last nSamples FEI readings, excluding extreme values ("trimmed mean")
//  result in integer Hz
int feiTrimMeanHz(uint8_t nSamples = FEI_HIST_SIZE, uint8_t trimPercent=20);
// As above, result in integer FSTEP units
int16_t feiTrimMean(uint8_t nSamples = FEI_HIST_SIZE, uint8_t trimPercent=20);

// Set frequency, argument is channel number (0 to NUMCHANS)
void setChan(uint8_t chan);

// Tune to a data channel, argument is INDEX in dataChans[] map!
//   (do not use a channel number as argument)
// To hop to a channel by number, use instead setChan() above
void setDataChanX(uint8_t dChanX);

// Tune to a control channel (next in sequence and increment)
// Call this repeatedly to traverse channel map in a scanning pattern
void setCtrlChan();

// Reset receiver soft buffers and state variables
// Call this after a successful object reception or transmission, 
//   or to bail out of a receive operation that failed partway
void resetState(bool dirty = false);

// Call from Arduino setup() to initialize radio and data structures
void gtmlabInit();

// Call from Arduino loop to perform receiving tasks
void gtmlabLoop();

// Check (from Arduino loop) if gtm main loop is in a busy state and
//  would prefer to not be held up by a lengthy non-gtm task
bool gtmlabBusy();

// called from radio receiver for each good packet received
int rxPacket(uint8_t * rxBuf, uint8_t rxLen, uint8_t uRSSI);

// called from main loop to check radio and perform RX tasks
bool gtmlabRxTask();

// called from main loop to check for TX tasks and execute the first in queue
bool gtmlabTxTask();

// Prepares the radio and state buffers for transmitting one or more packets
// There are a few things that need to be done before hitting TX on that radio
//   in order to make it a safe and seamless experience.
// This function performs all pre-flight steps, then sets the radio in TX mode
// The complement of this function, to be called at the END of the tx session,
//   is resetState(), which places the radio back in the default RX mode.
int txStart();

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
int txPacket(uint8_t *txBuf, uint8_t txLen, bool isCtrl);

// Takes a packet payload,
// prepends packet header, appends crc16 and reed-solomon code
// and then invokes txPacket() to SEND the packet over radio
// NOTE: Don't use this function directly unless you really need to;
//  the txSend(Ack|Sync|Msg) functions below are easier and safer
int txEncodeAndSend(uint8_t * pktBuf, uint8_t pktLen, uint8_t pktType);

// Send a SYNC packet (will set channel; expects TX mode on)
int txSendSync(uint8_t chIDX, uint8_t frags, uint8_t iniTTL, uint8_t curTTL);

// Send an ACK packet (will set channel; expects TX mode on)
int txSendAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);

// Send a TIME packet (will set channel; expects TX mode on)
int txSendTime(uint64_t time32);

// Send a message
// This is a complex action involving several packets
// (will set channel; expects TX mode on)
int txSendMsg(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL);

// Send ONE ACK or MSG packet, including TX mode switching
bool txSendAckOne(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);
bool txSendMsgOne(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL);

#endif // GTMRADIO_H
