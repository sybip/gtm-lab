//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// Board-specific peripherals such as GPS, PMU, screens etc
// Used for example by gtmAPI to query battery status etc
//

#include "peripheral.h"

#include <Wire.h>
#include "axp20x.h"  // AXP202X_Library by Lewis He (power mgmt)

// OLED screen library
#include "SSD1306Wire.h"  // ESP8266/ESP32 OLED Driver for SSD1306 by ThingPulse

#define TAG "PERIPH"

#define SSD1306_ADDRESS 0x3C

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

// safe defaults
#define GPS_SERIAL Serial1
#define GPS_SPEED 9600

// GPS related defines and variables
SFE_UBLOX_GNSS myGPS;

uint8_t hasScreen = 0;
uint8_t hasAXP192 = 0;
bool gpsAct = false;

AXP20X_Class axp;

// ADDRESS, SDA, SCL
// SDA and SCL usually populate automatically based on your board's pins_arduino.h
// e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
SSD1306Wire display(SSD1306_ADDRESS, SDA, SCL);


void scanI2C(void)
{
  uint8_t err, addr;
  for (addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      LOGI("I2C device found at address 0x%x", addr);
      if (addr == SSD1306_ADDRESS) {
        hasScreen = addr;
        LOGI("- SSD1306 OLED");
      }
      if (addr == AXP192_SLAVE_ADDRESS) {
        hasAXP192 = addr;
        LOGI("- AXP192 PMU");
      }
    }
  }
}


void boardInit(void)
{
  // Initialize I2C
  Wire.begin((int) SDA, (int) SCL);

  // Scan I2C bus for connected modules
  scanI2C();

  if (hasAXP192) {
    int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);
    if (ret != AXP_FAIL) {
      axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                     AXP202_VBUS_CUR_ADC1 |
                     AXP202_BATT_CUR_ADC1 |
                     AXP202_BATT_VOL_ADC1,
                     true);
      // axp.setTimer(1);
    } else {
        LOGW("AXP Power begin failed");
    }
  }

  if (hasScreen) {
    // initialize display
    display.init();    
  }
}


// BATTERY FUNCTIONS
// -----------------

// result in 0.1mV
uint16_t getBatteryVoltage()
{
  if (hasAXP192) {
    return round(axp.getBattVoltage() * 10);
  }
  return 0;
}


bool getBatteryCharging()
{
  if (hasAXP192) {
    return (axp.isChargeing());
  }
  return false;
}

// GPS FUNCTIONS
// -------------

// Attempt to connect to GPS via serial port; sometimes, the first attempt
//  may fail for no good reason, so we retry up to 3 times
// WARNING: every failed myGPS.begin() attempt will block for 3400ms!
bool gpsBegin(unsigned int portBaud = GPS_SPEED, uint8_t retries = 3)
{
  bool gpsGood = false;
#ifdef GPS_PIN_RX
  GPS_SERIAL.begin(portBaud, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);
#else
  GPS_SERIAL.begin(portBaud, SERIAL_8N1);
#endif  // GPS_PIN_RX

  delay(100);  // allow port to settle

  for (int i=0; i<retries; i++) {
    // try to init GPS (takes a long time to fail)
    if (myGPS.begin(GPS_SERIAL) == true) {
      gpsGood = true;
      break;
    }
    // delay(100 * (i+1));
  }
  if (gpsGood)
    LOGI("GPS Connected at %dbps", portBaud);
  else
    LOGW("GPS CONN FAILED at %dbps", portBaud);

  return(gpsGood);
}


// Attempt to recover a non-responding GPS module by brute-forcing
//  its port speed and issuing a factory reset
bool gpsRecover()
{
  bool gpsGood = false;

  // First, brute force port speed
  // FIXME this could be MASSIVELY improved by using an array of baudrates
  //  sorted by likelihood, instead of a dumb counter loop
  // (are we seriously testing 1800 bps before 38400? SO DUMB)
  int tBaud = 0;
  for (int i=0; i<10; i++) {
    tBaud = 1200 << i;
    if (gpsGood = gpsBegin(tBaud, 2))
      break;
    tBaud = 1800 << i;
    if (gpsGood = gpsBegin(tBaud, 2))
      break;
  }

  // Found the GPS? now reset it and reconnect at default speed
  if (gpsGood) {
    LOGW("GPS FOUND, issuing FACTORY RESET");
    myGPS.factoryReset();
    delay(1000);  // allow internals to settle
    gpsGood = gpsBegin(GPS_SPEED);
  }

  return(gpsGood);
}


// Initialize GPS connection, trying to recover from any known issues
//
void gpsInit()
{
  LOGI("GPS INIT");
  gpsAct = gpsBegin(GPS_SPEED);

  if (!gpsAct) {
    LOGW("GPS FAILED, attempting recovery; PLEASE WAIT");
    gpsAct = gpsRecover();
  }

  if (gpsAct) {
    LOGI("GPS ACTIVE");
    myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    myGPS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.powerSaveMode(false);
    myGPS.setAutoPVT(true);  // make GPS non-blocking
  } else {
    LOGW("GPS UNAVAILABLE");  // in spite of our best efforts
  }
}
