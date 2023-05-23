//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// +--------------+     +---------------+
// |   gtmNode    |     | gtmRadio      |
// +--------------+     +---------------+
// |   gtmlabInit >-----> gtmRadioInit  |
// |   gtmlabBusy >-----> gtmRadioBusy  |
// |              |     |               |
// |   gtmlabLoop >-----> gtmRadioRxTask|
// | processRxMsg <-----<-----|         |
// | processRxAck <-----<-----|         |
// |  handleRxERR <-----<-----'         |
// |              |     |               |
// | gtmlabTxTask->-----> gtmRadioTxMsg |
// |        `----->-----> gtmRadioTxAck |
// +--------------+     +---------------+

#ifndef GTMNODE_H
#define GTMNODE_H

#include <Arduino.h>

// Message class IDs
#define MSG_CLASS_P2P   0
#define MSG_CLASS_GROUP 1
#define MSG_CLASS_SHOUT 2
#define MSG_CLASS_EMERG 3


extern bool relaying;       // enable mesh relay function
extern bool noDeDup;
extern bool ezOutput;       // enable simple hexdump output of RX/TX events

// COUNTERS
// ACK packets received
extern uint32_t cntRxPktACKUni;  // unique
// ACK packets transmitted
extern uint32_t cntTxPktACKRel;  // relayed
// DATA objects received
extern uint32_t cntRxDataObjTot;  // total
extern uint32_t cntRxDataObjUni;  // unique
// DATA objects transmitted
extern uint32_t cntTxDataObjTot;  // total
extern uint32_t cntTxDataObjRel;  // relayed

// ported from gtmRadio, not sure if needed
extern uint16_t txInertia;    // INERTIA - millis to wait before TX
extern uint16_t txInerMAX;    // INERTIA - maximum value

// event handler functions
extern bool (* onRxMSG)(uint8_t *, uint16_t, uint8_t, uint8_t, uint8_t, uint16_t);
extern bool (* onRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
extern bool (* onTxMSG)(uint16_t);
extern bool (* onTxACK)(uint16_t);
extern bool (* onRxERR)(uint16_t);

// builtin event handlers
bool builtinRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);
bool builtinRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);
bool builtinRxERR(uint16_t error);
bool builtinTxMsgOK(uint16_t hashID);
bool builtinTxAckOK(uint16_t hashID);

// GTH16 hash algorithm (based on park-miller LCG)
uint16_t gtAlgoH16(uint8_t* str, size_t len);

// Calculates message hash using GTH16 algo
uint16_t msgHash16(uint8_t* mBuf);

// Check (from Arduino loop) if gtm main loop is in a busy state and
//  would prefer to not be held up by a lengthy non-gtm task
bool gtmlabBusy();

// Call from Arduino setup() to initialize radio and data structures
void gtmlabInit();

// Call from Arduino loop to perform receiving tasks
void gtmlabLoop();

// called from main loop to check for TX tasks and execute the first in queue
bool gtmlabTxTask();

// The functions below push an outbound object into the relevant
//  outbound queue, to be transmitted as soon as the channel is clear
// Applications requiring collision avoidance (really ALL production
//  applications) should use these functions to transmit messages
//  instead of calling txSend(Ack|Sync|Msg) directly

// Queue a message for sending (when channel is clear)
bool txEnQueueMSG(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL);

// Queue an ACK for sending (when channel is clear)
bool txEnQueueACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);

// Check (e.g from Arduino loop) if tx queue contains any objects
//   (which would be tx-ed on the next call to gtmTxTask)
bool txQueueEmpty();

bool processRxACK(uint16_t msgH16, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);
bool processRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);

// called from gtmRadio to report an RX error
void handleRxERR(uint16_t error);

#endif  // GTMNODE_H
