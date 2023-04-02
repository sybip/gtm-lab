//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2022 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#ifndef GTMXDISCOVER_H
#define GTMXDISCOVER_H

// Variables for DISCOVER feature
extern unsigned long discoverInit;  // When we started the test
extern unsigned long discoverSent;  // When finished sending probe packet
extern unsigned long discoverLast;  // When finished receiving latest echo
extern uint16_t discoverHash;    // Hash ID of the echo test
extern uint16_t discEchoCount;   // Number of echoes received

#define DISCOVER_MAXCOUNT 30
struct discEcho {
  unsigned long tStamp;
  uint8_t curTTL;
  uint8_t iniTTL;
  uint8_t uRSSI;
  uint16_t FEI;
};

extern discEcho discEchoList[];

void discoverStart();
void discoverFinish();

#endif // GTMXDISCOVER_H
