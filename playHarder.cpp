//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include <TimeLib.h>
#include "LoRaRegs.h"
#include "LoRaX.h"
#include "gtmRadio.h"
#include "gtmConfig.h"

// THE HARDCORE PLAYGROUND
// -----------------------
//  do you even lift? :)

// define USE_UBXGPS to enable and use the onboard u-blox GPS.
// Library "SparkFun u-blox GNSS" is required for GPS functions
// (install it in Arduino IDE using "Tools > Manage Libraries")
//
//#define USE_UBXGPS

#ifdef USE_UBXGPS
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

// safe defaults
#define GPS_SERIAL Serial1
#define GPS_SPEED 9600

#if BOARD_TYPE == 1
  // GPS connection for T-Beam board
  #define HAS_UBXGPS
  #define GPS_PIN_RX 34
  #define GPS_PIN_TX 12
#endif

// GPS related defines and variables
SFE_UBLOX_GNSS myGPS;

#endif  // USE_UBXGPS

#define PLAY_VER 2021031901   // Playground version

// GTA Message Body TLVs
#define MSGB_TLV_TYPE 0x01    // Message type, a %d string of a number(!)
#define MSGB_TLV_NICK 0x03    // Message sender nickname
#define MSGB_TLV_TEXT 0x04    // Message body text
#define MSGB_TLV_GDST 0x05    // Destination GID in group messages
#define MSGB_TLV_LCTN 0x06    // Location object
#define MSGB_TLV_PUBK 0xfc    // Public key object

// Serial port speed
#define SERIAL_SPEED_HIGH 1000000  // high speed - 1Mbps
#define SERIAL_SPEED_NORM 115200  // "normal" speed - 115200

// Logging functions and options
#define TAG "GTMLAB"

uint16_t appID = DFLT_APPID;  // defined in gtmConfig.h
uint8_t testITTL = 3;   // initial TTL for test messages
uint8_t testCTTL = 2;   // current TTL for test messages

// Assembles and sends a "shout" message with the supplied string as message body
// (thanks to https://gitlab.com/almurphy for the Arduino implementation)
int testShoutTx(char * msgBody, uint16_t msgLen, int argAppID=-1, bool compatGTA=true, bool direct=false)
{
  uint32_t time32 = now();
  uint8_t mData[300];  // message buffer (for a small message)
  uint16_t mPos = 0;
  uint8_t blobPos = 0;

  // Message class and App ID
  if (argAppID < 0) {
    argAppID = appID;  // use default
  }
  mData[mPos++] = MSG_CLASS_SHOUT;
  mData[mPos++] = (argAppID >> 8) & 0xff;  // AppID MSB
  mData[mPos++] = argAppID & 0xff;         // AppID LSB

  // No destination element, skip to HEAD element
  mData[mPos++] = 0xfb;  // Type of HEAD element: 0xFB
  mData[mPos++] = 0x10;  // Size of HEAD element: 16 bytes
  mData[mPos++] = 0;     // Body is not encrypted
  mData[mPos++] = 0;     // padding
  mData[mPos++] = 0;     // padding

  // sender GID, unused
  for (int i=0; i<6; i++) {
      mData[mPos++] = random(255);
  }

  // timestamp
  mData[mPos++] = (time32 >> 24) & 0xff;
  mData[mPos++] = (time32 >> 16) & 0xff;
  mData[mPos++] = (time32 >> 8) & 0xff;
  mData[mPos++] = time32 & 0xff;

  // Sequence numbers - set to 1 if unused
  mData[mPos++] = 0;  // seq0
  mData[mPos++] = 1;  // seq0
  mData[mPos++] = 1;  // seq1

  // blob starts here
  blobPos = mPos;

  if (compatGTA) {
    // GTA compatible - use TLV elements

    // MSGB_TLV_TYPE
    memcpy(mData+mPos, "\x01\x01\x30", 3);
    mPos += 3;

    // MSGB_TLV_NICK
    memcpy(mData+mPos, "\x03\x02\x3a\x29", 4);
    mPos += 4;

    // MSGB_TLV_TEXT
    mData[mPos++] = 4;
    mData[mPos++] = msgLen;
  }

  // Here comes the actual payload
  memcpy(mData+mPos, msgBody, msgLen);
  mPos += msgLen;

  // Almost DONE! now add a CRC16 to blob
  // (this is reqd by GTA and different to packet CRC)
  uint16_t msgCRC = 0;
  for (int i=blobPos; i<mPos; i++) {
      msgCRC = CRC16_add(mData[i], msgCRC);
  }
  mData[mPos++] = (msgCRC >> 8) & 0xff;
  mData[mPos++] = msgCRC & 0xff;

  // Check the total length: (BLOB+CRC+HEAD+2 < 256)?
  if ((mPos-blobPos) > (255-18)) {
    LOGW("Blob too big (%d > %d), THIS WILL FAIL!", (mPos-blobPos), (255-18));
  }

  LOGI("TX DATASIZE=%d", mPos);

  HEXD(TAG, mData, mPos);

  if (direct) {
    // SEND DIRECTLY (don't "Listen-Before")
    // fire up the TX
    txStart();
    // send the message (this will also generate and send the SYNC)
    txSendMsg(mData, mPos, testITTL, testCTTL);
    // finally return to RX mode
    resetState();  // return to RX mode with a clean slate
    LOGI("TX COMPLETE");

  } else {
    // SEND VIA NEW RINGBUFFER (Listen-Before etc)
    if (txEnQueueMSG(mData, mPos, testITTL, testCTTL)) {
      LOGI("TX QUEUED");
    } else {
      LOGI("TX DROPPED (buffer full)");
    }
  }

}


// Generate a text of random size and send it as a "shout" message
int testMessage(uint16_t msgLen)
{
  if (msgLen == 0)
    msgLen = random(230);

  char mBody[256] = {0};

  sprintf(mBody, "L=%03d:", msgLen);
  for (int i=6; i<msgLen; i++) {
    mBody[i] = 0x30 + (i % 10);
  }
  mBody[msgLen] = 0;  // C string termination

  testShoutTx(mBody, msgLen);
}


// how the GPS type is reported
#ifdef HAS_UBXGPS
  #define GPS_TYPE "hardware"
#else
  #define GPS_TYPE "emulated"
#endif

// ATAK PLI beacon code, stolen from:
// https://gitlab.com/almurphy/ESP32GTM/-/blob/master/examples/03-atak-beacon/03-atak-beacon.ino

#define BCAST_INTERVAL 60  // seconds

// CoT identity variables
char selfUUID[] = "gtm-lab-1234-1234";  // should be unique
char callSign[] = "GTM-LAB";            // should be personalized
char unitType[] = "a-f-G-U-C";  // friendly-ground-unit-combat
char unitTeam[] = "Red";        // Oh YES

// CoT position variables, populated with some dummy data
// (in production, these should be updated from the GPS receiver!)
int64_t gpsLAT = 51948900;     // microdeg
int64_t gpsLON = 4053500;      // microdeg
int64_t gpsHAE = 1000;         // millimeters
uint8_t gpsSIV = 1;   // satellites in view
uint8_t gpsDOP = 1;   // point dillution of precision
uint8_t gpsFix = 1;   // gps fix type
bool gpsAct = true;   // gps is ACTIVE

int testTakPLI()
{
  char gtmCoT[256] = {0};
  snprintf(gtmCoT, 200, "%s;%s;%s;%s;%.06f;%.06f;%.03f;%s;%d",
           selfUUID, unitType, callSign, 
           gpsFix ? "m-g" : ((gpsLON || gpsLAT) ? "h-e" : "h-g-i-g-o"),
           gpsLAT/1000000., gpsLON/1000000., gpsHAE/1000.,
           unitTeam, BCAST_INTERVAL);

  // Display, then broadcast the compressed CoT message
  LOGI("SENDING: %s", gtmCoT);

  // use ATAK appID, DO NOT include GTA COMPAT TLVs
  testShoutTx(gtmCoT, strlen(gtmCoT), 0xd8ff, false);
}


#ifdef HAS_UBXGPS
void gpsInit()
{
#ifdef GPS_PIN_RX
  GPS_SERIAL.begin(GPS_SPEED, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);
#else
  GPS_SERIAL.begin(GPS_SPEED, SERIAL_8N1);
#endif  // GPS_PIN_RX

  delay(100);  // allow port to settle

  gpsAct = false;
  for (int i=0; i<3; i++) {
    // try to init GPS
    if (myGPS.begin(GPS_SERIAL) == true) {
      gpsAct = true;
      break;
    }
    delay(100 * (i+1));
  }

  if (gpsAct) {
    LOGI("GPS ACTIVE");
    myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    myGPS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.powerSaveMode(false);
  }
}
#endif  // HAS_UBXGPS


// Playground initialization, to be called from Arduino init()
void playInit()
{
  LOGI("playground version: %d, gps type: %s", PLAY_VER, GPS_TYPE);

#ifdef HAS_UBXGPS
  // GPS initialization
  gpsInit();
#endif  // HAS_UBXGPS
}

// Console input handler
// executed for each line received on the serial console
// - if first character is '!', treat the line as a command
// - otherwise, transmit it in the body of a "shout" message
int conExec(char *conBuf, uint16_t conLen)
{
  char hexBuf[3] = {0};
  uint8_t wReg = 0;
  uint8_t wVal = 0;
  uint8_t xChan = 0;
  uint8_t mData[256];  // message buffer (for a small message)

  if (conLen && (conBuf[0] != '!')) {
    // take text from conBuf and shout it
    testShoutTx(conBuf, conLen);
    return(0);
  }

  // implement a (very basic) command console
  // see playHarder.md for command reference
  //
  if (conBuf[1] == 'l') {
    if (conBuf[2] == 'v') {
      logLevel = ESP_LOG_VERBOSE;
      LOGI("SET_LOG: VERBOSE");
    } else if (conBuf[2] == 'd') {
      logLevel = ESP_LOG_DEBUG;
      LOGI("SET_LOG: DEBUG");
    } else if (conBuf[2] == 'i') {
      logLevel = ESP_LOG_INFO;
      LOGI("SET_LOG: INFO");
    } else if (conBuf[2] == 'w') {
      logLevel = ESP_LOG_WARN;
      LOGI("SET_LOG: WARN");
    } else if (conBuf[2] == 'e') {
      logLevel = ESP_LOG_ERROR;
      LOGI("SET_LOG: ERROR");
    }
    esp_log_level_set("*", logLevel);

  } else if (conBuf[1] == 'm') {
    scanning = false;
    LOGI("SCANNING is now OFF ('!h1' to resume)");
    if (conBuf[2] == '0') {
      LOGI("SETMODE: SLEEP");
      LoRa.writeRegister(REG_OP_MODE, MODE_SLEEP);
      // all op-modes raise "Mode Ready" bit when ready, 
      //   except sleep mode, where all ISR bits are zero
      while (LoRa.readRegister(REG_IRQ_FLAGS_1)) { ; }
    } else if (conBuf[2] == '1') {
      LOGI("SETMODE: STDBY");
      LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);
      while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80)) { ; }      
    } else if (conBuf[2] == 'r') {
      LOGI("SETMODE: RX");
      LoRa.writeRegister(REG_OP_MODE, MODE_RX_CONTINUOUS);
      while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80)) { ; }      
    } else if (conBuf[2] == 't') {
      LOGI("SETMODE: TX");
      LoRa.writeRegister(REG_OP_MODE, MODE_TX);
      while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80)) { ; }      
    } else {
      LOGI("UNKMODE");      
    }

  } else if (conBuf[1] == 'h') {
    if (conBuf[2] == '0') {
      scanning = false;
      holdchan = false;
    } else if (conBuf[2] == '1') {
      scanning = true;
      holdchan = false;
    } else if (conBuf[2] == '2') {
      scanning = false;
      holdchan = true;
    }
    LOGI("SCANNING is now %s", (scanning ? "ON":"OFF"));
    LOGI("HOLDCHAN is now %s", (holdchan ? "ON":"OFF"));

  } else if (conBuf[1] == 'c') {
      memcpy(hexBuf, conBuf+2, 2);
      xChan = strtoul(hexBuf, NULL, 10) & 0xff;
      scanning = false;
      // Range check
      if (xChan >= (curRegSet->dChanNum + curRegSet->cChanNum)) {
        LOGW("Channel out of range (0-%d)", (curRegSet->dChanNum + curRegSet->cChanNum - 1));
      } else {
        LOGI("Set Channel: %d", xChan);
        setChan(xChan);
      }

  } else if (conBuf[1] == 's') {
    if (conBuf[2] == 'd') {
      memcpy(hexBuf, conBuf+3, 2);
      txPackDelay = strtoul(hexBuf, NULL, 10) & 0xff;
      LOGI("PACKDELAY now %dms", txPackDelay);
    } else if (conBuf[2] == 'g') {
      switch(conBuf[3]) {
        case '1':
          LOGI("SETGEO: US");
          curRegSet = &(regSets[0]);
          break;
        case '2':
          LOGI("SETGEO: EU");
          curRegSet = &(regSets[1]);
          break;
        case '4':
        case '5':
          LOGI("SETGEO: AU");
          curRegSet = &(regSets[2]);
          break;
        case '8':
          LOGI("SETGEO: JP");
          curRegSet = &(regSets[3]);
          break;
        default:
          LOGW("SETGEO - NOT FOUND!");
      }
      resetState();

    } else if (conBuf[2] == 'p') {
      memcpy(hexBuf, conBuf+3, 2);
      uint8_t txPower = strtoul(hexBuf, NULL, 10) & 0xff;
      if (txPower > MAX_TX_POWER)
        txPower = MAX_TX_POWER;
      LoRa.setTxPower(txPower);
      LOGI("SET TX Power: %ddBm", txPower);

    } else if (conBuf[2] == 'a') {
      appID = strtoul(conBuf+3, NULL, 16) & 0xffff;
      LOGI("SET APPID: %04x", appID);

    } else if (conBuf[2] == 'r') {
      if (conBuf[3] == '0') {
        relaying = false;
      } else if (conBuf[3] == '1') {
        relaying = true;
      }
      LOGI("RELAYING is now %s", (relaying ? "ON":"OFF"));

    } else if (conBuf[2] == 't') {
      if (conLen == 5) {
        memcpy(hexBuf, conBuf+3, 2);
        wVal = strtoul(hexBuf, NULL, 16) & 0xff;
        testITTL = (wVal >> 4) & 0x0f;
        testCTTL = wVal & 0x0f;
        LOGI("Test iniTTL=%d, curTTL=%d", testITTL, testCTTL);
      }

    } else if (conBuf[2] == 's') {
      if (conBuf[3] == 'n') {
        LOGW("CHANGE console speed to %dbps NOW", SERIAL_SPEED_NORM);
        LOGW("------");
        delay(10);
        Serial.end();
        delay(10);
        Serial.begin(SERIAL_SPEED_NORM);
      } else if (conBuf[3] == 'h') {
        LOGW("CHANGE console speed to %dbps NOW", SERIAL_SPEED_HIGH);
        LOGW("------");
        delay(10);
        Serial.end();
        delay(10);
        Serial.begin(SERIAL_SPEED_HIGH);
      }
      while (!Serial);
      while (Serial.available())
        Serial.read();

#ifdef HAS_UBXGPS
    } else if (conBuf[2] == 'c') {
        if (conBuf[3] == 'g') {  // setclock from gps
          if (gpsAct) {
            LOGI("SETCLOCK from GPS:");
            LOGI("OLD TIME: %04d-%02d-%02d %02d:%02d:%02d", 
                  year(), month(), day(), hour(), minute(), second());
            setTime(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond(),
                    myGPS.getDay(), myGPS.getMonth(), myGPS.getYear());
            LOGI("NEW TIME: %04d-%02d-%02d %02d:%02d:%02d", 
                  year(), month(), day(), hour(), minute(), second());
          } else {
            LOGW("GPS time unavailable");
          }
        }
#endif  // HAS_UBXGPS
    }

  } else if (conBuf[1] == 'd') {
    // VIEW
    if (conBuf[2] == 'a') {
      // READ ALL REGISTERS
      dumpRegisters(-1);  // to stdout
    } else if (conBuf[2] == 'i') {
      // READ INTERRUPT_REGISTERS
      printf(BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\n", 
             BYTE_TO_BINARY(LoRa.readRegister(REG_IRQ_FLAGS_1)),
             BYTE_TO_BINARY(LoRa.readRegister(REG_IRQ_FLAGS_2)));

    } else if (conBuf[2] == 'f') {
      // READ FIFO
      while (!((LoRa.readRegister(REG_IRQ_FLAGS_2)) & 0x40)) {
        printf("%02x ",LoRa.readRegister(REG_FIFO));
      }
      printf("\n");

    } else if (conBuf[2] == 'c') {
      // uint32_t cntRxPktAll = cntRxPktSYNC + cntRxPktDATA + cntRxPktACK + cntRxPktTIME;
      // uint32_t cntRxErrAll = cntErrPRESTALL + cntErrPKTSTALL + cntErrLOSTSYNC + cntErrREEDSOLO + cntErrCRC16BAD;

      // DUMP ALL COUNTERS
      printf("SYS.UPTIME: %.3fs\n", millis()/1000.);
      printf("RX PACKETS:\n- SYNC: %d\n- DATA: %d\n-  ACK: %d\n- TIME: %d\n", 
             cntRxPktSYNC, cntRxPktDATA, cntRxPktACK, cntRxPktTIME);
      printf("TX PACKETS:\n- SYNC: %d\n- DATA: %d\n-  ACK: %d\n- TIME: %d\n", 
             cntTxPktSYNC, cntTxPktDATA, cntTxPktACK, cntTxPktTIME);
      printf("RX OBJECTS:\n- DATA: %d (%d unique)\n-  ACK: %d (%d unique)\n", 
             cntRxDataObjTot, cntRxDataObjUni, cntRxPktACK, cntRxPktACKUni);
      printf("TX OBJECTS:\n- DATA: %d (%d own, %d relay)\n-  ACK: %d (%d own, %d relay)\n", 
             cntTxDataObjTot, (cntTxDataObjTot - cntTxDataObjRel), cntTxDataObjRel, 
             cntTxPktACK, (cntTxPktACK - cntTxPktACKRel), cntTxPktACKRel);
      printf("ERRORS:\n");
      printf("- PRESTALL: %d\n", cntErrPRESTALL);
      printf("- PKTSTALL: %d\n", cntErrPKTSTALL);
      printf("- LOSTSYNC: %d\n", cntErrLOSTSYNC);
      printf("- REEDSOLO: %d\n", cntErrREEDSOLO);
      printf("- CRC16BAD: %d\n", cntErrCRC16BAD);

    } else if (conBuf[2] == 's') {
      // READ STATE VARIABLES
      printf("PLAY_VER: %d\n", PLAY_VER);
      printf("GPS_TYPE: " GPS_TYPE "\n");
      printf("LOGLEVEL: %d\n", logLevel);
      printf("SCANNING: %s\n", scanning ? "ON":"OFF");
      printf("HOLDCHAN: %s\n", holdchan ? "ON":"OFF");
      printf("RECVDATA: %s\n", recvData ? "ON":"OFF");
      printf("INTXMODE: %s\n", inTXmode ? "ON":"OFF");
      printf("CURRCHAN: %d\n", currChan);
      printf("RELAYING: %s\n", relaying ? "ON":"OFF");
      printf("BASEFREQ: %d\n", curRegSet->baseFreq);
      printf("CHANSTEP: %d\n", curRegSet->chanStep);
      printf("CCHANNUM: %d\n", curRegSet->cChanNum);
      printf("DCHANNUM: %d\n", curRegSet->dChanNum);
      printf("TestITTL: %d\n", testITTL);
      printf("TestCTTL: %d\n", testCTTL);
      printf("MY_APPID: 0x%04x\n", appID);
      printf("SYS_TIME: %04d-%02d-%02d %02d:%02d:%02d\n", 
             year(), month(), day(), hour(), minute(), second());
#ifdef HAS_UBXGPS
      printf("UBLOXGPS: %s\n", gpsAct ? "ON":"OFF");
#endif  // HAS_UBXGPS

#ifdef HAS_UBXGPS
    } else if (conBuf[2] == 'g') {
      if (gpsAct) {
        gpsFix = myGPS.getFixType();

        printf("GPS ONLINE\n");
        printf("GPS_TIME: %04d-%02d-%02d %02d:%02d:%02d (%s)\n", 
          myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(),
          myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond(),
          myGPS.getTimeValid() ? "VAL" : "INV");

        printf("GPS_LAT : %d\nGPS_LON : %d\nGPS_HAE : %d\n",
          myGPS.getLatitude(),  // *10E7
          myGPS.getLongitude(), // *10E7
          myGPS.getAltitude());  // mm

        printf("GPS_DOP : %d\nGPS_SIV : %d\nGPS_FIX : %d\n", 
          myGPS.getPDOP(),      // *100
          myGPS.getSIV(),
          gpsFix);

      } else {
        printf("GPS OFFLINE\n");
      }
#endif  // HAS_UBXGPS

    } else if (conBuf[2] == 'r') {
      if (conLen == 5) {
        memcpy(hexBuf, conBuf+3, 2);
        wReg = strtoul(hexBuf, NULL, 16) & 0xff;
        wVal = LoRa.readRegister(wReg);
        printf("0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN"\n", wReg, wVal, BYTE_TO_BINARY(wVal));
      }
    }

  } else if (conBuf[1] == 'w') {
    // WRITE register wXXYY
    memcpy(hexBuf, conBuf+2, 2);
    wReg = strtoul(hexBuf, NULL, 16) & 0xff;
    memcpy(hexBuf, conBuf+4, 2);
    wVal = strtoul(hexBuf, NULL, 16) & 0xff;
    LOGI("Write: %02x = %02x", wReg, wVal);
    LoRa.writeRegister(wReg, wVal);

  } else if (conBuf[1] == 't') {
    if (conBuf[2] == 'd') {
      // TEST/DIRECT - send an ACK packet DIRECTLY
      txStart();    // fire up transmitter
      txSendAck(random(65535), 1, testITTL, testCTTL);
      resetState(); // return to RX mode

    } else if (conBuf[2] == 'a') {
      // TEST/ACK - test ACK sending from main loop with LBT
      uint16_t hashID = random(65535);
      // if a hashID was provided (!taXXXX), use that
      if (conLen == 7) {
        hashID = strtoul(conBuf+3, NULL, 16) & 0xffff;
      }
      if (txEnQueueACK(hashID, 1, testITTL, testCTTL)) {
        LOGI("ACK QUEUED");
      } else {
        LOGI("ACK DROPPED (buffer full)");
      }

    } else if (conBuf[2] == 't') {
      // TEST/TIME - send a TIME packet DIRECTLY
      // use system time if valid, or a hardcoded constant otherwise
      txStart();    // fire up transmitter
      if (year() > 2020) {  // system time presumed valid
        txSendTime(now());
      } else {  // system time invalid, send a bogon (sorry!)
        txSendTime(0x60501337);
      }
      resetState(); // return to RX mode

    } else if (conBuf[2] == 'x') {
      // TEST/HEXMSG
      int i;

      // unhexlify message into temp buffer mData
      for (i = 0; i < ((conLen-3)>>1); i++) {
        memcpy(hexBuf, conBuf+3+(i<<1), 2);
        mData[i] = strtoul(hexBuf, NULL, 16) & 0xff;
      }
      if (txEnQueueMSG(mData, i, testITTL, testCTTL)) {
        LOGI("TX QUEUED");
      } else {
        LOGI("TX DROPPED (buffer full)");
      }

    } else if (conBuf[2] == 'm') {
      // TEST/MSG - send a random message
      if (conLen == 5) {
        // message length was provided
        memcpy(hexBuf, conBuf+3, 2);
        wVal = strtoul(hexBuf, NULL, 16) & 0xff;
        LOGI("LEN: %d", wVal);
        testMessage(wVal);
      } else {
        testMessage(0);
      }

    } else if (conBuf[2] == 'k') {
      // TEST/ATAK - send a random ATAK PLI message
      testTakPLI();
    }

  } else {
    LOGW("UNKNOWN COMMAND");
  }
}
