//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include "gtmConfig.h"
#include "gtmRadio.h"

// Serial console buffer
char conBuf[256];
uint16_t conLen=0;

// playHarder.cpp is our debug console aka the "playground"
//  send commands received via usb serial, and stuff happens
int conExec(char *conBuf, uint16_t conLen);


void setup()
{
  Serial.begin(115200);     // initialize serial console
  while (!Serial);          // recommended if USB serial

  esp_log_level_set("*", VERBOSITY);

  // initialize radio and data structures
  gtmlabInit();
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
      conExec(conBuf, conLen);  // in playHarder.cpp
      conLen = 0;
    } else {
      conBuf[conLen++] = conOne;
    }
  }
}
