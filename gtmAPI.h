//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// Relation between code modules:
//
// +------------+     +------------+
// | gtmBLE     |     | gtmAPI     |           \|/
// +------------+     +------------+            |
// | gtmCmdExec >-----> apiCmdExec |     +--------------+
// | gtmCmdResp <-----< apiCmdResp |     | gtmNode+Radio|
// | gtmBLEFlag <-----< apiSetFlag |     +-------------+
// +------------+     |   apiRxMsg <-----< onRxMsg      |
//                    |   apiRxAck <-----< onRxAck      |
//                    |   gtmTxMsg >-----> txEnQueueMSG |
//                    |   gtmTxAck >-----> txEnQueueACK |
//                    +------------+     +--------------+

#ifndef GTMAPI_H
#define GTMAPI_H

#include <Arduino.h>

// API result codes (using upper 2 bits)
#define RES_SUCCESS 0x40
#define RES_FAILURE 0xc0

// UI EVENTS
#define UI_EVT_ATTENTION 0x01  // on original GTM, blink LED 3 times

// from GTM32
#define MESG_TLV_0x04  0x04    // (R,T) Undoc, required in some cases
#define MESG_TLV_DATA  0x05    // (R,T) Main section: sender, body, payloads
#define MESG_TLV_DEST  0x06    // (R,T) Message class and dest (GID or all)
#define MESG_TLV_HOPS  0x20    // (R,A) Rx'd msg hops count
#define MESG_TLV_DLR   0x21    // (R) Delivery report (created by device)
#define MESG_TLV_TTL   0x22    // (T) Message TTL (set by sender app)


struct gtGID {
  uint8_t cls;
  uint16_t app;
  uint8_t gid[6];
  uint8_t tag;  
  uint16_t gth;  
};

#define LARGER_THAN_ANY_MESSAGE 300  // FIXME or EXPLAIN

struct inBoxItem {
  unsigned long tStamp;
  uint8_t msgObj[LARGER_THAN_ANY_MESSAGE];
  uint16_t msgLen;
};

extern unsigned long apiCmdRejectMap;  // commands will be ignored, returning a negative result
extern unsigned long apiCmdIgnoreMap;  // commands will be ignored, returning a positive result

// API inputs from Radio - suitable as drop-in handlers for onRxMsg and onRxAck callbacks
//bool gtmMsgRx2API(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI);
//bool gtmAckRx2API(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI);
bool apiRxMsg(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);
bool apiRxAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI);

// API msg output to Radio
bool gtmMsgAPI2Tx(uint8_t * buf, uint16_t len);

// Hooks for calls to radio TX routines for MSG and ACK objects
extern bool (* gtmTxMsg)(uint8_t *, uint16_t, uint8_t, uint8_t);
extern bool (* gtmTxAck)(uint16_t, uint8_t, uint8_t, uint8_t);

// Built-in default (no-op) handlers for radio hooks
bool nopTxMsg(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL);
bool nopTxAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);

// API command input from BLE (or other client frontend)
bool apiCmdExec(uint8_t cmd, uint8_t seq, uint8_t * buf, size_t len);
// API response callbacks to BLE (or, again, other client frontend)
extern void (* apiCmdResp)(uint8_t, uint8_t, uint8_t *, size_t);
extern void (* apiSetFlag)(uint8_t);

// API UI hook
extern void (* apiUIEvent)(uint8_t);

// Built-in default (no-op) handlers for API BLE and UI callbacks
void nopCmdResp(uint8_t cmd, uint8_t seq, uint8_t* data, size_t len);
void nopSetFlag(uint8_t st);
void nopUIEvent(uint8_t evt);

#endif  // GTMAPI_H
