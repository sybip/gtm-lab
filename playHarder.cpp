//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include <TimeLib.h>
#include "LoRaRegs.h"
#include "LoRaX.h"
#include "gtmConfig.h"
#include "gtmRadio.h"

#include "gtmXDiscover.h"

// optional, read .h file comments before including
#include "gtmXAdHocCal.h"

#include "peripheral.h"

// THE HARDCORE PLAYGROUND
// -----------------------
//  do you even lift? :)

// define USE_GTMBLE to include and use the BLE GTM API server
// Library gtmBLE is required
#ifdef USE_GTMBLE
#include "gtmBLE.h"
#endif  // USE_GTMBLE

#ifdef ESP32
#include "esp_system.h"
#endif

#define PLAY_VER 2023040401   // Playground version

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

uint8_t chanRSSI[64] = { 0 };   // shouldn't be that many channels, max known is 51
char symbRSSI[64] = { 0 };      // symbolic representation of RSSI
char symbList[] = { '#', '=', '-', ' ' };

// Assembles and sends a "shout" message with the supplied string as message body
// (thanks to https://gitlab.com/almurphy for the Arduino implementation)
void testShoutTx(char * msgBody, uint16_t msgLen, int argAppID=-1, bool compatGTA=true, bool direct=false)
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
void testMessage(uint16_t msgLen)
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

void testTakPLI()
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


// Playground initialization, to be called from Arduino init()
void playInit()
{
  LOGI("playground version: %d, gps type: %s", PLAY_VER, GPS_TYPE);

#ifdef HAS_UBXGPS
  // GPS initialization
  gpsInit();
#endif  // HAS_UBXGPS

#ifdef ADHOC_CALIBRATION
  adCalibStart();
#endif  // ADHOC_CALIBRATION
}

// called from arduino loop
void playLoop()
{
  // any discovery in progress?
  if(discoverInit && (millis() - discoverInit > 5000)) {
    discoverFinish();
  }

#ifdef ADHOC_CALIBRATION
  // any calibration in progress?
  if (calibRunning) {
    adCalibCheck();
  }
#endif  // ADHOC_CALIBRATION
}


// Console input handler
// executed for each line received on the serial console
// - if first character is '!', treat the line as a command
// - otherwise, transmit it in the body of a "shout" message
int playExec(char *conBuf, uint16_t conLen)
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
      switch(conBuf[3]) {
        case 's':
          memcpy(hexBuf, conBuf+4, 2);
          txSyncDelay = strtoul(hexBuf, NULL, 10) & 0xff;
          LOGI("SYNCDELAY now %dms", txSyncDelay);
          break;
        case 'd':
          memcpy(hexBuf, conBuf+4, 2);
          txPackDelay = strtoul(hexBuf, NULL, 10) & 0xff;
          LOGI("PACKDELAY now %dms", txPackDelay);
          break;
        default:
          LOGW("NOT FOUND");
      }
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

    } else if (conBuf[2] == 'f') {  // Frequency correction
      switch(conBuf[3]) {
        case 'a':                   // Software AFC on/off
          if (conBuf[4] == '0') {
            softAFC = false;
          } else if (conBuf[4] == '1') {
            softAFC = true;
          }
          LOGI("SOFT_AFC is now %s", (softAFC ? "ON":"OFF"));
          break;
        case 't':
          memcpy(hexBuf, conBuf+4, 3);
          uint16_t tFeiThre;
          tFeiThre = strtoul(hexBuf, NULL, 10) & 0xffff;
          if (tFeiThre)
            feiThre = tFeiThre;
          LOGI("FEI_THRE is now (+/-)%d", feiThre);
          break;
        case 'c':
          memcpy(hexBuf, conBuf+4, 3);
          uint16_t tFreqCorr;
          tFreqCorr = strtoul(hexBuf, NULL, 10) & 0xffff;
          if (tFreqCorr)
            freqCorr = tFreqCorr;
          LOGI("FREQCORR is now %d (%d Hz)", freqCorr, freqCorr * 61);  // FSTEP unints
          fcorrRegTemp = getRadioTemp();
          break;
#ifdef ADHOC_CALIBRATION
        case 'z':                   // Recalibrate FCORR
          if (!calibRunning) {
            LOGI("RECALIBRATING");
            adCalibStart();
          } else {
            LOGW("Calibration already IN PROGRESS");
          }
          break;
#endif  // ADHOC_CALIBRATION
        default:
          LOGW("NOT FOUND");
      }

    } else if (conBuf[2] == 'p') {
      memcpy(hexBuf, conBuf+3, 2);
      uint8_t txPower = strtoul(hexBuf, NULL, 10) & 0xff;
      if (txPower > MAX_TX_POWER)
        txPower = MAX_TX_POWER;
      gtmSetTxPower(txPower);
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

    } else if (conBuf[2] == 'o') {
      if (conBuf[3] == '0') {
        ezOutput = false;
      } else if (conBuf[3] == '1') {
        ezOutput = true;
      }
      LOGI("Easy Output is now %s", (ezOutput ? "ON":"OFF"));

#ifdef HAS_UBXGPS
    } else if (conBuf[2] == 'c') {
        if (conBuf[3] == 'g') {  // setclock from gps
          if (gpsAct) {
            LOGI("SETCLOCK from GPS:");
            LOGI("OLD TIME: %04d-%02d-%02d %02d:%02d:%02d", 
                  year(), month(), day(), hour(), minute(), second());
            myGPS.setAutoPVT(false);  // make GPS blocking
            myGPS.getSecond();        // flush old data
            setTime(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond(),
                    myGPS.getDay(), myGPS.getMonth(), myGPS.getYear());
            LOGI("NEW TIME: %04d-%02d-%02d %02d:%02d:%02d", 
                  year(), month(), day(), hour(), minute(), second());
            myGPS.setAutoPVT(true);  // make GPS non-blocking
          } else {
            LOGW("GPS time unavailable");
          }
        }
#endif  // HAS_UBXGPS

#ifdef USE_GTMBLE
    } else if (conBuf[2] == 'b') {
      // status
      if (conBuf[3] == '0') {
        if (gtmBLEStatus()) {
          LOGI("BLE: turning OFF");
          gtmBLEStop();
        } else {
          LOGI("BLE is already OFF");
        }
      } else if (conBuf[3] == '1') {
        if (! gtmBLEStatus()) {
          LOGD("BLE: turning ON");
          gtmBLEInit();
        } else {
          LOGI("BLE is already ON");
        }
      }
#endif  // USE_GTMBLE
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

    } else if (conBuf[2] == 'q') {
      // DUMP RF SETTINGS
      printf("FREQ: %d\n", getFrequency());
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
      printf("RX ERRORS:\n");
      printf("- PRESTALL: %d\n", cntErrPRESTALL);
      printf("- PKTSTALL: %d\n", cntErrPKTSTALL);
      printf("- LOSTSYNC: %d\n", cntErrLOSTSYNC);
      printf("- REEDSOLO: %d\n", cntErrREEDSOLO);
      printf("- CRC16BAD: %d\n", cntErrCRC16BAD);
      printf("- MISSCHAN: %d\n", cntErrMISSCHAN);

      printf("TX ERRORS:\n");
      printf("- TXJAMMED: %d\n", cntErrTXJAMMED);

      printf("LBT COUNTERS:\n");
      printf("- CTRLFREE: %d\n", cntCChFree);
      printf("- CTRLBUSY: %d\n", cntCChBusy);
      printf("- DATAFREE: %d\n", cntDChFree);
      printf("- DATABUSY: %d\n", cntDChBusy);

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
      printf("CHANTIME: %d ms\n", millis() - chanTimer);
      printf("LASTRECV: %d ms\n", millis() - lastRadioRx);
      printf("LASTXMIT: %d ms\n", millis() - lastRadioTx);
      printf("INERTIA : %d (max=%d)\n", txInertia, txInerMAX);
      printf("LBT_THRE: -%d dBm\n", lbtThreDBm);
      printf("PKTDELAY: SYNC=%d DATA=%d\n", txSyncDelay, txPackDelay);
      printf("RELAYING: %s\n", relaying ? "ON":"OFF");
      printf("BASEFREQ: %d\n", curRegSet->baseFreq);
      printf("CHANSTEP: %d\n", curRegSet->chanStep);
      printf("CCHANNUM: %d\n", curRegSet->cChanNum);
      printf("DCHANNUM: %d\n", curRegSet->dChanNum);
      printf("TX_POWER: %d dBm\n", gtmTxPower);
      printf("TestITTL: %d\n", testITTL);
      printf("TestCTTL: %d\n", testCTTL);
      printf("MY_APPID: 0x%04x\n", appID);
      printf("NO_DEDUP: %s\n", noDeDup ? "ON":"OFF");
      printf("EZOUTPUT: %s\n", ezOutput ? "ON":"OFF");
      printf("SYS_TIME: %04d-%02d-%02d %02d:%02d:%02d\n", 
             year(), month(), day(), hour(), minute(), second());
#ifdef HAS_UBXGPS
      printf("UBLOXGPS: %s\n", gpsAct ? "ON":"OFF");
#endif  // HAS_UBXGPS
#ifdef USE_GTMBLE
      switch(gtmBLEStatus()) {
        case BLE_SERVICE_CONNECTED:
          printf("BLE_SERV: CONNECTED\n");
          break;
        case BLE_SERVICE_ACCEPTING:
          printf("BLE_SERV: ACCEPTING\n");
          break;
        case BLE_SERVICE_DISABLED:
          printf("BLE_SERV: DISABLED\n");
          break;
      }
#endif  // USE_GTMBLE

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
          myGPS.getLatitude(),  // *1E7
          myGPS.getLongitude(), // *1E7
          myGPS.getAltitude());  // mm

        printf("GPS_DOP : %d\nGPS_SIV : %d\nGPS_FIX : %d\n", 
          myGPS.getPDOP(),      // *100
          myGPS.getSIV(),
          gpsFix);

      } else {
        printf("GPS OFFLINE\n");
      }
#endif  // HAS_UBXGPS

#ifdef USE_GTMBLE
    } else if (conBuf[2] == 'b') {
      // FIXME redundant, make function?
      switch(gtmBLEStatus()) {
        case BLE_SERVICE_CONNECTED:
          printf("BLE_SERV: CONNECTED\n");
          break;
        case BLE_SERVICE_ACCEPTING:
          printf("BLE_SERV: ACCEPTING\n");
          break;
        case BLE_SERVICE_DISABLED:
          printf("BLE_SERV: DISABLED\n");
          break;
      }
      printf("LAST_CHG: %ds ago\n", tLastConnChg);
      printf("CONNECTS: %d\n", cntConnects);
      printf("COMMANDS: %d\n", cntCommands);
      printf(" SUCCESS: %d\n", cntRSuccess);
      printf(" FAILURE: %d\n", cntRFailure);
      printf(" NEITHER: %d\n", cntRNeither);

#endif  // USE_GTMBLE

    } else if (conBuf[2] == 'r') {
      if (conLen == 5) {
        memcpy(hexBuf, conBuf+3, 2);
        wReg = strtoul(hexBuf, NULL, 16) & 0xff;
        wVal = LoRa.readRegister(wReg);
        printf("0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN"\n", wReg, wVal, BYTE_TO_BINARY(wVal));
      }

    } else if (conBuf[2] == 'w') {
      // sWeep all channels
      unsigned long scanEnd = millis() + 500;  // (ms)
      if (conLen == 5) {
        memcpy(hexBuf, conBuf+3, 2);
        scanEnd = millis() + strtoul(hexBuf, NULL, 10) * 1000;
      }

      uint8_t oChan = currChan;
      while (millis() < scanEnd) {
        for (uint8_t chan = 0; chan < curRegSet->tChanNum; chan++) {
            setChan(chan);
            delay(2);  // ms
            chanRSSI[chan] = LoRa.readRegister(REG_RSSI_VALUE);
            symbRSSI[chan] = symbList[chanRSSI[chan]>>6];
            symbRSSI[chan+1] = 0;
        }
        printf("%d [%s]\n", millis(), symbRSSI);
      }
      setChan(oChan);

    } else if (conBuf[2] == 't') {
      // time tracking
      uint16_t tPeriod = ttGetPeriod();
      uint16_t lPeriod = tPeriod;
      if (lPeriod)
        lPeriod--;
      else
        lPeriod = TIMETRACK_PERIODS - 1;
      unsigned long tPTime = ttPeriodTime();
      if (tPTime == 0)   // avoid zero division
        tPTime = 1;
      printf("PERIOD: %d/%d\n", tPeriod, TIMETRACK_PERIODS);
      printf("PERIOD TIME: %d/%d\n", tPTime, TIMETRACK_MILLIS);
      printf("- TIME TX : %d (%.2f%%/%.2f%%)\n", TTRK[tPeriod].timeTX,
              100.0*TTRK[tPeriod].timeTX/tPTime, 100.0*TTRK[lPeriod].timeTX/TIMETRACK_MILLIS);
      printf("- TIME RX : %d (%.2f%%/%.2f%%)\n", TTRK[tPeriod].timeRX,
              100.0*TTRK[tPeriod].timeRX/tPTime, 100.0*TTRK[lPeriod].timeRX/TIMETRACK_MILLIS);
      printf("- TIME LBT: %d (%.2f%%/%.2f%%)\n", TTRK[tPeriod].timeLBT,
              100.0*TTRK[tPeriod].timeLBT/tPTime, 100.0*TTRK[lPeriod].timeLBT/TIMETRACK_MILLIS);
      printf("- TIME EVT: %d (%.2f%%/%.2f%%)\n", TTRK[tPeriod].timeEvt,
              100.0*TTRK[tPeriod].timeEvt/tPTime, 100.0*TTRK[lPeriod].timeEvt/TIMETRACK_MILLIS);

    } else if (conBuf[2] == 'e') {
      // FEI history
      int feiFirst = feiHistPos - feiHistLen;
      int feiSum = 0;
      if (feiFirst < 0)
        feiFirst += FEI_HIST_SIZE;
      for (uint8_t ix = 0; ix < feiHistLen; ix++) {
        int16_t feiVal = (int16_t) feiHist[(feiFirst + ix) % FEI_HIST_SIZE];
        if (conBuf[3] == 'v') {
          printf("%d\n", feiVal);
        }
        feiSum += feiVal;
      }
      //if (feiHistLen)
      //  printf("FEI_AVG=%.2f\n", 1.0 * feiSum / feiHistLen);
      printf("FREQ_COR: %d\n", freqCorr);
      printf("TEMP_COR: %d\n", fcorrRegTemp);
      printf("TEMP_NOW: %d\n", getRadioTemp());
#ifdef ADHOC_CALIBRATION
      printf("TEMP_CAL: %d\n", calibRegTemp);
#endif
      printf("NSAMPLES: %d\n", feiHistLen);
      printf("FEI_MEAN: %d\n", feiTrimMean());
    }
    printf("---\n");

  } else if (conBuf[1] == 'w') {
    // WRITE register wXXYY (or wsXXYY to put radio in standby mode first)
    uint8_t numPos = 2;  // position of numbers
    if (conBuf[2] == 's') {
      numPos++;
    }
    memcpy(hexBuf, conBuf+numPos, 2);
    wReg = strtoul(hexBuf, NULL, 16) & 0xff;
    memcpy(hexBuf, conBuf+numPos+2, 2);
    wVal = strtoul(hexBuf, NULL, 16) & 0xff;
    LOGI("Write: %02x = %02x", wReg, wVal);
    if (conBuf[2] == 's') {  // write in standby mode (sometimes required)
      uint8_t oldMode = LoRa.readRegister(REG_OP_MODE);
      // put radio in standby mode
      LoRa.writeRegister(REG_OP_MODE, MODE_STDBY);
      while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80)) { ; }
      LoRa.writeRegister(wReg, wVal);
      LoRa.writeRegister(REG_OP_MODE, oldMode);
      while (!(LoRa.readRegister(REG_IRQ_FLAGS_1) & 0x80)) { ; }
    } else    // just write
      LoRa.writeRegister(wReg, wVal);

  } else if (conBuf[1] == 'f') {  // FILTERING
    if (conBuf[2] == 'd') {
      if (conBuf[3] == '0') {
        noDeDup = true;
      } else if (conBuf[3] == '1') {
        noDeDup = false;
      }
      LOGI("Duplicate Filter is now %s", (noDeDup ? "OFF":"ON"));
    }

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

    } else if (conBuf[2] == 'e') {
      // TEST/ECHO
      discoverStart();

    } else if (conBuf[2] == 'k') {
      // TEST/ATAK - send a random ATAK PLI message
      testTakPLI();
    }

  } else if (conBuf[1] == 'z') {
    // Reset (Zero) some internal variables
    if (conBuf[2] == 'c') {  // counters (from !dc)
      gtmlabResetCounts();
      LOGI("ALL COUNTERS RESET");

    } else if (conBuf[2] == 'e') {  // FEI history (from !de)
      feiHistLen = feiHistPos = 0;
      LOGI("FEI HISTORY RESET");

#ifdef ESP32
    } else if (conBuf[2] == 'z') {  // reboot
      LOGI("REBOOT");
      esp_restart();
#endif
    } else {
      LOGW("UNKNOWN COMMAND");
    }

  } else {
    LOGW("UNKNOWN COMMAND");
  }

  return(0);
}
