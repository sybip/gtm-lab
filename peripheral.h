//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// Board-specific peripherals such as GPS, PMU, screens etc
// Used for example by gtmAPI to query battery status etc
//

#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#include "axp20x.h"
#include "SSD1306Wire.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

#include "gtmConfig.h"
#include "gtmRadio.h"

#define LED_OFF LOW
#define LED_ON HIGH

#if BOARD_TYPE == 1
  // GPS connection for T-Beam board
  #define HAS_UBXGPS
  #define GPS_PIN_RX 34
  #define GPS_PIN_TX 12
  #define PIN_BUTTON 38
  #define PIN_REDLED 4
  // these are different on my tbeam 1.0!
  // update them for compatibility
  #ifdef LED_BUILTIN
    #undef LED_BUILTIN
  #endif
  #ifdef KEY_BUILTIN
    #undef KEY_BUILTIN
  #endif
  #define LED_BUILTIN PIN_REDLED
  #define KEY_BUILTIN PIN_BUTTON
  #define LED_OFF HIGH
  #define LED_ON LOW

#elif BOARD_TYPE == 2 or BOARD_TYPE == 3
  #ifdef KEY_BUILTIN
    #undef KEY_BUILTIN
  #endif
  #define PIN_BUTTON 0
  #define KEY_BUILTIN PIN_BUTTON
#endif


extern AXP20X_Class axp;
extern SSD1306Wire display;
extern SFE_UBLOX_GNSS myGPS;

extern uint8_t hasScreen;
extern uint8_t hasAXP192;
extern bool gpsAct;

void boardInit(void);
void gpsInit(void);

uint16_t getBatteryVoltage();  // result in 0.1mV
bool getBatteryCharging();

#endif  // PERIPHERAL_H
