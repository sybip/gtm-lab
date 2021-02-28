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

#define PLAY_VER 2021022803   // Playground version

// GTA Message Body TLVs
#define MSGB_TLV_TYPE 0x01    // Message type, a %d string of a number(!)
#define MSGB_TLV_NICK 0x03    // Message sender nickname
#define MSGB_TLV_TEXT 0x04    // Message body text
#define MSGB_TLV_GDST 0x05    // Destination GID in group messages
#define MSGB_TLV_LCTN 0x06    // Location object
#define MSGB_TLV_PUBK 0xfc    // Public key object

// Logging functions and options
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  // DO NOT EDIT THIS LINE
#define TAG "GTMLAB"
#include "esp_log.h"

#define LOGE( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   TAG, format, ##__VA_ARGS__)
#define LOGW( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    TAG, format, ##__VA_ARGS__)
#define LOGI( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    TAG, format, ##__VA_ARGS__)
#define LOGD( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   TAG, format, ##__VA_ARGS__)
#define LOGV( format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, format, ##__VA_ARGS__)

uint16_t appID = DFLT_APPID;  // defined in gtmConfig.h

// Assembles and sends a "shout" message with the supplied string as message body
// (thanks to https://gitlab.com/almurphy for the Arduino implementation)
int testShoutTx(char * msgBody, uint16_t msgLen, bool direct = false)
{
  uint32_t time32 = now();
  uint8_t mData[300];  // message buffer (for a small message)
  uint16_t mPos = 0;
  uint8_t blobPos = 0;

  // Message class and App ID
  mData[mPos++] = MSG_CLASS_SHOUT;
  mData[mPos++] = (appID >> 8) & 0xff;  // AppID MSB
  mData[mPos++] = appID & 0xff;         // AppID LSB

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

  // GTA Message body TLVs
  blobPos = mPos;

  // MSGB_TLV_TYPE
  memcpy(mData+mPos, "\x01\x01\x30", 3);
  mPos += 3;

  // MSGB_TLV_NICK
  memcpy(mData+mPos, "\x03\x02\x3a\x29", 4);
  mPos += 4;

  // MSGB_TLV_TEXT
  uint8_t msgSize = strlen(msgBody);

  // clamp message size to 220 bytes
  if (msgSize > 220) {
    LOGW("Message too long, may fail");
  }

  mData[mPos++] = 4;
  mData[mPos++] = msgSize;
  memcpy(mData+mPos, msgBody, msgSize);
  mPos += msgSize;

  // Almost DONE! now add a CRC16 to blob
  // (this is reqd by GTA and different to packet CRC)
  uint16_t msgCRC = 0;
  for (int i=blobPos; i<mPos; i++) {
      msgCRC = CRC16_add(mData[i], msgCRC);
  }
  mData[mPos++] = (msgCRC >> 8) & 0xff;
  mData[mPos++] = msgCRC & 0xff;

  LOGI("TX DATASIZE=%d", mPos);

  ESP_LOG_BUFFER_HEXDUMP(TAG, mData, mPos, ESP_LOG_DEBUG);

  if (direct) {
    // SEND DIRECTLY (don't "Listen-Before")
    // fire up the TX
    txStart();
    // send the message (this will also generate and send the SYNC)
    txSendMsg(mData, mPos, 3, 2);
    // finally return to RX mode
    resetState();  // return to RX mode with a clean slate
    LOGI("TX COMPLETE");

  } else {
    // SEND VIA NEW RINGBUFFER (Listen-Before etc)
    if (txEnQueueMSG(mData, mPos, 3, 2)) {
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

  char mBody[256];

  sprintf(mBody, "L=%03d:", msgLen);
  for (int i=6; i<msgLen; i++) {
    mBody[i] = 0x30 + (i % 10);
  }
  mBody[msgLen] = 0;  // C string termination

  testShoutTx(mBody, msgLen);
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

  if (conBuf[0] != '!') {
    // take text from conBuf and shout it
    testShoutTx(conBuf, conLen);
    return(0);
  }

  // implement a (very basic) command console
  // available commands:
  //  !lv = set logging level to VERBOSE
  //  !ld = set logging level to DEBUG
  //  !li = set logging level to INFO
  //  !lw = set logging level to WARNING
  //  !le = set logging level to ERROR
  //  -----
  //  !m0 = set radio SLEEP mode
  //  !m1 = set radio STANDBY mode
  //  !mr = set radio RX mode
  //  !mt = set radio TX mode
  //    (all !m commands will also pause chan scanning)
  //  -----
  //  !sgX = set geopolitical region to X (1,2,4,8)
  //  !spDD = set TX power dBm (DD in decimal 00 - 20)
  //  !sr0 = set mesh relay function OFF
  //  !sr1 = set mesh relay function ON
  //  !saXXXX = set App ID to XXXX (in HEX)
  //  -----
  //  !da = dump ALL radio registers
  //  !di = dump ISR registers
  //  !df = dump entire FIFO content
  //      (use !dr00 to read one FIFO byte)
  //  !drXX = dump radio register XX (in HEX)
  //  !ds = dump state variables (incomplete)
  //  -----
  //  !wXXYY = write to radio register XX value YY
  //      (XX and YY in HEX)
  //  -----
  //  !cDD = change channel (DD in decimal)
  //  -----
  //  !h0 = control chan scanning pause
  //  !h1 = control chan scanning resume
  //  -----
  //  !td = transmit a random ACK packet directly
  //  !tt = transmit a TIME packet directly
  //  !ta = transmit a random ACK packet using queue
  //     !taXXXX = specify hash ID (in HEX)
  //  !tm = transmit a random shout using queue
  //     !tmXX = specify body size (in HEX)
  //  !tx = transmit a data object supplied in HEX
  //
  if (conBuf[1] == 'l') {
    if (conBuf[2] == 'v') {
      esp_log_level_set("*", ESP_LOG_VERBOSE);
      LOGI("SET_LOG: VERBOSE");
    } else if (conBuf[2] == 'd') {
      esp_log_level_set("*", ESP_LOG_DEBUG);
      LOGI("SET_LOG: DEBUG");
    } else if (conBuf[2] == 'i') {
      esp_log_level_set("*", ESP_LOG_INFO);
      LOGI("SET_LOG: INFO");
    } else if (conBuf[2] == 'w') {
      esp_log_level_set("*", ESP_LOG_WARN);
      LOGI("SET_LOG: WARN");
    } else if (conBuf[2] == 'e') {
      esp_log_level_set("*", ESP_LOG_ERROR);
      LOGI("SET_LOG: ERROR");
    }

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
    } else if (conBuf[2] == '1') {
      scanning = true;
    }
    LOGI("SCANNING is now %s", (scanning ? "ON":"OFF"));

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

    } else if (conBuf[2] == 's') {
      // READ STATE VARIABLES
      printf("PLAY_VER: %d\n", PLAY_VER);
      printf("SCANNING: %s\n", scanning ? "ON":"OFF");
      printf("RECVDATA: %s\n", recvData ? "ON":"OFF");
      printf("INTXMODE: %s\n", inTXmode ? "ON":"OFF");
      printf("CURRCHAN: %d\n", currChan);
      printf("RELAYING: %s\n", relaying ? "ON":"OFF");
      printf("BASEFREQ: %d\n", curRegSet->baseFreq);
      printf("CHANSTEP: %d\n", curRegSet->chanStep);
      printf("CCHANNUM: %d\n", curRegSet->cChanNum);
      printf("DCHANNUM: %d\n", curRegSet->dChanNum);
      printf("MY_APPID: 0x%04x\n", appID);

    } else if (conBuf[2] == 'r') {
      memcpy(hexBuf, conBuf+3, 2);
      wReg = strtoul(hexBuf, NULL, 16) & 0xff;
      wVal = LoRa.readRegister(wReg);
      printf("0x%02x | 0x%02x | "BYTE_TO_BINARY_PATTERN"\n", wReg, wVal, BYTE_TO_BINARY(wVal));
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
      txSendAck(random(65535), 1, 3, 2);
      resetState(); // return to RX mode

    } else if (conBuf[2] == 'a') {
      // TEST/ACK - test ACK sending from main loop with LBT
      uint16_t hashID = random(65535);
      // if a hashID was provided, use that
      if (conLen == 7) {
        hashID = strtoul(conBuf+3, NULL, 16) & 0xffff;
      }
      if (txEnQueueACK(hashID, 1, 3, 2)) {
        LOGI("ACK QUEUED");
      } else {
        LOGI("ACK DROPPED (buffer full)");
      }

    } else if (conBuf[2] == 't') {
      // TEST/TIME - send a TIME packet DIRECTLY
      txStart();    // fire up transmitter
      txSendTime(0x60201337);
      resetState(); // return to RX mode

    } else if (conBuf[2] == 'x') {
      // TEST/HEXMSG
      int i;

      // unhexlify message into temp buffer mData
      for (i = 0; i < ((conLen-3)>>1); i++) {
        memcpy(hexBuf, conBuf+3+(i<<1), 2);
        mData[i] = strtoul(hexBuf, NULL, 16) & 0xff;
      }
      if (txEnQueueMSG(mData, i, 3, 2)) {
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
    }

  } else {
    LOGW("UNKNOWN COMMAND");
  }
}
