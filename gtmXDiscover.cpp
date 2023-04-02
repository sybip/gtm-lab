//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2022 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

// DISCOVER extension for GTM LAB
// ------------------------------
// Enumerates the unique GTM devices in range, with timing and RSSI
//
// This feature operates invisible to the network users, relying only on
//  standard GTM network functionality and does not require the cooperation
//  of the other network nodes
//
// Theory of operation:
// 1) transmit a small packet (type ACK) with a chosen TTL
// 2) listen for echoes (retransmissions) of the packet
//    each received echo identifies a unique node
// 3) for in-range nodes (hops=1), RSSI indicates link quality
//    hops>1 indicates an asymmetrical path, RSSI not reliable
//

#include "gtmRadio.h"
#include "gtmConfig.h"
#include "gtmXDiscover.h"

#define TAG "DISCVR"

unsigned long discoverInit = 0;  // When we started the test
unsigned long discoverSent = 0;  // When finished sending probe packet
unsigned long discoverLast = 0;  // When finished receiving latest echo
uint16_t discoverHash = 0;    // Hash ID of the echo test
uint16_t discEchoCount = 0;   // Number of echoes received

bool (* oldRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t);
bool (* oldTxACK)(uint16_t);

bool oldNoDeDup;

// imported from playHarder (should #ifdef this dependency)
extern uint8_t testITTL;   // initial TTL for test messages
extern uint16_t currFei;
discEcho discEchoList[DISCOVER_MAXCOUNT];


// simple builtin ACK handler
bool discRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  if (hashID == discoverHash) {
    // our ID has been echoed
    if (discEchoCount < DISCOVER_MAXCOUNT) {
      // record an echoResp
      discEchoList[discEchoCount].tStamp = millis();
      discEchoList[discEchoCount].curTTL = curTTL;
      discEchoList[discEchoCount].iniTTL = iniTTL;
      discEchoList[discEchoCount].uRSSI = uRSSI;
      discEchoList[discEchoCount].FEI = FEI;
    }
    discEchoCount++;
    discoverLast = millis();
    LOGI("DISCOVER(%04x) RX:%d TTL=%d/%d RSSI=%d FEI=%d", hashID, 
          discEchoCount, curTTL, iniTTL, (uRSSI >> 1), (int16_t) currFei);
    return true;
  }

  // not our ID, pass-through to old handler
  return oldRxACK(hashID, hops, iniTTL, curTTL, uRSSI, FEI);
}


// simple builtin TX ACK succcess handler
bool discTxACK(uint16_t hashID)
{
  if (hashID == discoverHash) {
    // our ID has been sent
    discoverSent = millis();
    LOGI("DISCOVER(%04x) SENT", hashID);
    return true;
  }

  // not our ID, pass-through to old handler
  return oldTxACK(hashID);
}


// initiate DISCOVER (will change some settings)
void discoverStart()
{
  // save old handler and set custom ones
  oldRxACK = onRxACK;
  oldTxACK = onTxACK;
  oldNoDeDup = noDeDup;

  onRxACK = discRxACK;
  onTxACK = discTxACK;
  noDeDup = true;

  // generate a random ID for the test
  discoverHash = random(0xffff);

  // send with CTTL=ITTL
  if (txEnQueueACK(discoverHash, 1, testITTL, testITTL)) {
    LOGI("ACKPROBE QUEUED");
  } else {
    LOGI("ACKPROBE DROPPED (buff)");
  }
  discoverInit = millis();
  discoverSent = discoverLast = 0;
  discEchoCount = 0;
}


// finish DISCOVER (will restore settings changed at beginning of test)
void discoverFinish()
{
  LOGI("DISCOVER(%04x) TOTAL=%d", discoverHash, discEchoCount);
  // restore handlers
  onRxACK = oldRxACK;
  onTxACK = oldTxACK;
  noDeDup = oldNoDeDup;
  discoverInit = 0;
}
