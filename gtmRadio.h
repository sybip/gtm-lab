//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#ifndef GTMRADIO_H
#define GTMRADIO_H

#include <Arduino.h>

// Radio packet types
#define PKT_TYPE_TIME 0
#define PKT_TYPE_SYNC 1
#define PKT_TYPE_DATA 2
#define PKT_TYPE_ACK  3

// Message class IDs
#define MSG_CLASS_P2P   0
#define MSG_CLASS_GROUP 1
#define MSG_CLASS_SHOUT 2
#define MSG_CLASS_EMERG 3

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


// regional radio frequency and channel settings
struct regSet {
  uint32_t baseFreq;      // base frequency
  uint32_t chanStep;      // channel step
  uint8_t cChanNum;       // number of control channels
  uint8_t cChanMap[3];    // map of control channels
  uint8_t dChanNum;       // number of data channels
  uint8_t dChanMap[48];   // map of data channels
};


// state variables, from gtmRadio.cpp
extern bool scanning;
extern bool inTXmode;
extern bool recvData;       // if true, we are on a data chan
extern uint8_t currChan;    // current channel NUMBER
extern uint8_t currDChIdx;  // current data chan INDEX in map
extern uint8_t currCChIdx;  // current ctrl chan INDEX in map
extern bool relaying;       // enable mesh relay function
extern regSet regSets[];
extern regSet * curRegSet;
extern uint8_t txSyncDelay;   // millis to wait between sync packet and first data packet
extern uint8_t txPackDelay;   // millis to wait between data packets

// event handler functions
extern bool (* onRxMSG)(uint8_t *, uint16_t, uint8_t, uint8_t, uint8_t);
extern bool (* onRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t);

// builtin event handlers
bool builtinRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t RSSI);
bool builtinRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t RSSI);

// additive CRC16 function
uint16_t CRC16_add(uint8_t b, uint16_t crc = 0);

// GTH16 hash algorithm (based on park-miller LCG)
uint16_t gtAlgoH16(uint8_t* str, size_t len);

// Calculates message hash using GTH16 algo
uint16_t msgHash16(uint8_t* mBuf);

// Dump all radio registers, in hex and binary (datasheet for details)
// Argument: ESP log level, -1 for printf
void dumpRegisters(int logLevel);

// Look up a hash in one of the hash ringbuffers
bool inRingBuf(uint16_t needle, uint16_t *haystack);

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
void resetState();

// Call from Arduino setup() to initialize radio and data structures
void gtmlabInit();

// Call from Arduino loop to perform receiving tasks
void gtmlabLoop();

// called from radio receiver for each good packet received
int rxPacket(uint8_t * rxBuf, uint8_t rxLen);


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

// The functions below push an outbound object into the relevant
//  outbound queue, to be transmitted as soon as the channel is clear
// Applications requiring collision avoidance (really ALL production
//  applications) should use these functions to transmit messages
//  instead of calling txSend(Ack|Sync|Msg) directly

// Queue a message for sending (when channel is clear)
bool txEnQueueMSG(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL);

// Queue an ACK for sending (when channel is clear)
bool txEnQueueACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);

#endif // GTMRADIO_H
