//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// gtmMsgAPI2Tx code based on https://gitlab.com/almurphy/ESP32GTM
//   ESP32GTM is Copyright 2020 by Alec Murphy, MIT licensed
//

#include "gtmAPI.h"
#include "gtmRadio.h"

// Prefer to not include gtmBLE.h, in order to avoid BLE entanglement
//   (gtmAPI is intended to remain client connection agnostic)
// Instead, the glueing to gtmBLE should be done in the main .ino by 
//   using the hook functions provided in both gtmAPI and gtmBLE

// Peripheral support is used mainly to check battery levels and charging
//   status, however it will slightly increase the code size
// #include "peripheral.h"  // board-specific functions

#define TAG "GTMAPI"

#include "esp_log.h"

#define LOG_ ESP_LOG_LEVEL_LOCAL

#define INBOX_SIZE 32   // keep small to be able to test overflows etc

// sends data from server to client
void (* apiCmdResp)(uint8_t, uint8_t, uint8_t *, size_t) = nopCmdResp;

// sends status flag to client (gtmBLESetST);
void (* apiSetFlag)(uint8_t) = nopSetFlag;

// generate a UI event on the device, such as LED blinking
void (* apiUIEvent)(uint8_t) = nopUIEvent;

// Hooks for calls to radio TX routines for MSG and ACK objects
bool (* gtmTxMsg)(uint8_t *, uint16_t, uint8_t, uint8_t) = nopTxMsg;
bool (* gtmTxAck)(uint16_t, uint8_t, uint8_t, uint8_t) = nopTxAck;

inBoxItem inBoxData[INBOX_SIZE];
uint8_t inBoxHead = 0, inBoxTail = 0;

uint16_t inBoxMsgs()
{
  uint16_t c = 0;
  for (int i = 0; i < INBOX_SIZE; i++)
    if (inBoxData[i].msgLen)
      c++;
  return c;
}

// Bitmapped (1<<cmd) command permission lists
//   start empty and populate in main
unsigned long apiCmdRejectMap = 0;  // command will be ignored, returning a negative result
unsigned long apiCmdIgnoreMap = 0;  // command will be ignored, returning a positive result

gtGID myGIDSet[32] = {0};
uint8_t myGIDNum = 0;

// hard-coded GTM sysinfo block
// Most GTM apps will poll the sysinfo service to check battery status and other
//  system health data, and may freak out if we don't return a valid response
// Ideally, we should construct this block with actual and up-to-date information,
//  however in a minimal case we can just send this block to keep the ball in play
//
uint8_t sysInfoData[] = { 
  0x00,       // UNDOC (maybe MSB of unread below?)
  0x42,       // unread msgs
  0xa3, 0x81, // batt voltage
  0x00, 0x22, // PA temp
  0x00,       // undoc bitmap of error flags
  0x01, 0x01, 0x08,  // firm ver: 1.1.8
  0x00, 0x20, // SILABS temp
  0x00,       // undoc
  0x4d, 0x59, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30,  // serial: MY76543210
  0x00,       // is charging?
  0x06,       // hw ver: 6
  0x01,       // led on
  0xff, 0xff  // boot ver
};  // LEN = 28


// Converts a MSG object from api format to air format and TRANSMITS it
bool gtmMsgAPI2Tx(uint8_t * buf, uint16_t len)
{
  // use a 16-bit counter to be able to go over 255 (in case of TLV error) without overrunning
  uint16_t pos = 0; 
  uint8_t msgClass = 0;
  uint16_t msgAppID = 0;
  uint32_t msgTStamp = 0;
  uint32_t msgFromHi = 0, msgFromLo = 0;
  uint32_t msgDestHi = 0, msgDestLo = 0;
  uint8_t msgBlobPos = 0;
  uint8_t msgBlobLen = 0;
  uint8_t msgTTL = 3;

  uint8_t tBuf[256] = {0};
  uint8_t tPos = 0;

  // Message objects have a TLV structure
  // Parse this structure to expose the main elements
  while(pos < len) {
    LOGD("TLV TYPE=0x%02x, LEN=%d", buf[pos], buf[pos+1]);
    switch(buf[pos]) {
      case MESG_TLV_DEST:
        msgClass = buf[pos+2];
        msgAppID = (buf[pos+3]<<8) + buf[pos+4];

        // Two types of TLV_DEST - short (3 bytes) and long (10 bytes)
        if (buf[pos+1] == 10) {
          // long TLV_DEST, Class-App-GID-Tag
          msgDestHi = (buf[pos+5]<<8) + buf[pos+6];
          msgDestLo = (buf[pos+7]<<24) + (buf[pos+8]<<16) + (buf[pos+9]<<8) + buf[pos+10];
          // msgDest = (msgDestHi << 32) | msgDestLo;
          LOGD(" [DEST] Class=%d, AppID=%04x, Dest=0x%04x%08x, Tag=%0x02x", 
                    msgClass, msgAppID, msgDestHi, msgDestLo, buf[pos+11]);
        } else {
          // short TLV_DEST, Class-App
          LOGD(" [DEST] Class=%d, AppID=%04x", msgClass, msgAppID);
        }
        // FIXME copy to tbuf, assumes correct order
        memcpy(tBuf + tPos, &buf[pos+2], buf[pos+1]);
        tPos += buf[pos+1];
        break;

      case MESG_TLV_HOPS:
        LOGD(" [HOPS] Hops=%d, RSSI=%d", buf[pos+2], buf[pos+3]);
        break;

      case MESG_TLV_TTL:
        msgTTL = buf[pos+2];
        LOGD(" [TTL ] TTL=%d", msgTTL);
        break;

      case MESG_TLV_DATA:
        // Expecting DATA value to begin with 0xFB 0x10; anything else confuses us
        if ((buf[pos+2] != 0xfb) || (buf[pos+3] != 0x10)) {
          LOGD(" [DATA] UNKNOWN FORMAT");
          break;  // bail out early
        }

        // from pos+4 start parsing FB section
        // "BQLHB", cryptFlag, fromGID, tStamp, seqNo0, seqNo1
        // process sender field in 32-bit chunks to avoid Arduino headache
        msgFromHi = (buf[pos+5]<<24) + (buf[pos+6]<<16) + (buf[pos+7]<<8) + buf[pos+8];
        msgFromLo = (buf[pos+9]<<24) + (buf[pos+10]<<16) + (buf[pos+11]<<8) + buf[pos+12];

        // 32-bit unix timestamp
        msgTStamp = (buf[pos+13]<<24) + (buf[pos+14]<<16) + (buf[pos+15]<<8) + buf[pos+16];

        // The FB section of the DATA element has a standard format
        //   however, the rest of the element is an application specific
        //   blob, followed by a (2-byte) CRC16X
        msgBlobLen = buf[pos+1] -18 -2;  // Len from TLV minus sizeof FB header and CRC

        LOGD(" [DATA] Crypt=%d, From=0x%04x%08x, TStamp=%d, Len=%d", 
                  buf[pos+4], msgFromHi, msgFromLo, msgTStamp, msgBlobLen);

        // FIXME copy to tbuf, assumes correct order
        memcpy(tBuf + tPos, &buf[pos+2], buf[pos+1]);
        tPos += buf[pos+1];

        // Stop processing, proprietary formats ahead
        break;

      default:
        // we don't need to support every single possible element type
        LOGD(" [OTHER]");
        break;
    }

    // Finished processing element, advance read position
    pos += buf[pos+1] + 2;  // Len + T/L bytes
  }

  if (gtmTxMsg(tBuf, tPos, msgTTL, msgTTL)) {
    LOGI("TX QUEUED");
    return true;
  } else {
    LOGI("TX DROPPED (buffer full?)");
    return false;
  }
}


// Converts a received MSG object from air format to api format
bool apiRxMsg(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  uint16_t pos=0; 
  uint8_t msgClass = 0;
  uint16_t msgAppID = 0;
  uint32_t msgTStamp = 0;
  uint32_t msgFromHi = 0, msgFromLo = 0;
  uint32_t msgDestHi = 0, msgDestLo = 0;
  uint8_t msgBlobPos = 0;
  uint8_t msgBlobLen = 0;

  inBoxItem * ib = &inBoxData[inBoxHead];

  if (ib->msgLen) {
    // inbox is full; can decide to overwrite or drop message
    // we overwrite for now
    LOGW("INBOX overflow (head=%d, tail=%d)", inBoxHead, inBoxTail);
    // alternative, to drop message, just return at this point

    // if we overwrite, advance read pointer
    inBoxTail = (inBoxTail+1) % INBOX_SIZE;
  }
  LOGI("SAVEMSG @%d", inBoxHead);

  ib->msgLen = 0;

  msgClass = mBuf[pos];
  msgAppID = (mBuf[pos+1]<<8) + mBuf[pos+2];

  // DEST element is first
  ib->msgObj[ib->msgLen++] = MESG_TLV_DEST;
  uint8_t destSize = 10;
  if ((msgClass == 2) || (msgClass == 3)) {  // SHOUT and EMERG have short DEST
    destSize = 3;
  }
  ib->msgObj[ib->msgLen++] = destSize;
  memcpy(&ib->msgObj[ib->msgLen], &mBuf[pos], destSize);
  ib->msgLen += destSize;
  pos += destSize;

  // TLV4 element is next, P2P msgs only
  if (msgClass == 0) {
    ib->msgObj[ib->msgLen++] = 0x04;
    ib->msgObj[ib->msgLen++] = 0x03;
    memcpy(&ib->msgObj[ib->msgLen], &mBuf[pos], 3);
    pos += 3;
  }

  // DATA element
  ib->msgObj[ib->msgLen++] = MESG_TLV_DATA;
  ib->msgObj[ib->msgLen++] = mLen - pos;
  memcpy(&ib->msgObj[ib->msgLen], &mBuf[pos], mLen - pos);
  ib->msgLen += (mLen - pos);

  // HOPS element
  ib->msgObj[ib->msgLen++] = MESG_TLV_HOPS;
  ib->msgObj[ib->msgLen++] = 2;
  ib->msgObj[ib->msgLen++] = iniTTL - curTTL;
  ib->msgObj[ib->msgLen++] = uRSSI;

  // advance write pointer
  inBoxHead = (inBoxHead+1) % INBOX_SIZE;

  LOGI("MSGQLEN=%d", inBoxMsgs());

  // set status flag
  // FIXME may cause BLE activity and related debug messages in the middle
  //  of ezOutput lines (RX_MSG and RX_ACK specifically)
  apiSetFlag(1);

  return true;
}


// Converts a received ACK object from air format to api format
bool apiRxAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  inBoxItem * ib = &inBoxData[inBoxHead];

  if (ib->msgLen) {
    // inbox is full; can decide to overwrite or drop message
    LOGW("INBOX overflow (head=%d, tail=%d)", inBoxHead, inBoxTail);
    // if we overwrite, advance read pointer
    inBoxTail = (inBoxTail+1) % INBOX_SIZE;
  }
  LOGI("SAVEACK @%d", inBoxHead);

  // construct an ACK API object consisting of a single TLV
  ib->msgLen = 0;
  ib->msgObj[ib->msgLen++] = MESG_TLV_DLR;
  ib->msgObj[ib->msgLen++] = hops;
  ib->msgObj[ib->msgLen++] = (hashID >> 8) & 0xff;
  ib->msgObj[ib->msgLen++] = hashID & 0xff;

  // advance write pointer
  inBoxHead = (inBoxHead+1) % INBOX_SIZE;

  LOGI("MSGQLEN=%d", inBoxMsgs());

  // set status flag
  apiSetFlag(1);

  return true;
}

void nopCmdResp(uint8_t cmd, uint8_t seq, uint8_t* data, size_t len)
{ }

void nopSetFlag(uint8_t st)
{ }

void nopUIEvent(uint8_t evt)
{ }

bool nopTxMsg(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL)
{ return true; }

bool nopTxAck(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{ return true; }


bool apiCmdExec(uint8_t cmd, uint8_t seq, uint8_t * buf, size_t len)
{
  uint8_t tBuf[4] = { 0 };
  uint16_t batt_dmV = 0;

  cmd &= 0x3f;  // LS 10 bits for the opcode

  // Look up reject and ignore lists, and stop further processing if found
  if (apiCmdRejectMap & (1 << cmd)) {
    LOGI("CMD REJECTED");
    apiCmdResp(cmd | RES_FAILURE, seq, NULL, 0);
    return false;  // command was not executed
  }

  if (apiCmdIgnoreMap & (1 << cmd)) {
    LOGI("CMD IGNORED");
    apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
    return true;  // (pretend that) command was executed
  }

  switch (cmd) {
    case 0x00:  // blinken
      apiUIEvent(UI_EVT_ATTENTION);   // send to UI
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x01:  // set_gid
      ESP_LOG_BUFFER_HEXDUMP("SETGID", buf, len, ESP_LOG_INFO);
      if (len == 10) {
        myGIDSet[myGIDNum].cls = buf[0];
        myGIDSet[myGIDNum].app = (buf[1] << 8) + buf[2];
        myGIDSet[myGIDNum].tag = buf[9];
        memcpy(myGIDSet[myGIDNum].gid, &buf[3], 6);
        myGIDSet[myGIDNum].gth = gtAlgoH16(buf, 9);
        myGIDNum++;  // FIXME check for overflow
      } else {
        LOGW("Unsupp GID len=%d", len);
      }
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x02:  // set_pub (what how?)
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x03:  // sendmsg - response is "33 01 ff"
      // what was the negative response again?
      tBuf[0] = 0x33;
      tBuf[1] = 0x01;
      tBuf[2] = 0xff;
      gtmMsgAPI2Tx(buf, len);
      apiCmdResp(cmd | RES_SUCCESS, seq, tBuf, 3);
      break;

    case 0x04:  // sysinfo
      #ifdef PERIPHERAL_H
      // (FIXME) patch battery voltage in sysinfo block
      batt_dmV = getBatteryVoltage();
      sysInfoData[2] = (batt_dmV >> 8) & 0xff;
      sysInfoData[3] = batt_dmV & 0xff;

      // (FIXME) patch "batt charging" in sysinfo block
      if (getBatteryCharging()) {
        sysInfoData[23] = 1;
      } else {
        sysInfoData[23] = 0;
      }
      #endif  // PERIPHERAL_H

      // (FIXME) patch msgqlen in sysinfo block
      sysInfoData[1] = inBoxMsgs() & 0xff;

      apiCmdResp(cmd | RES_SUCCESS, seq, sysInfoData, 28);
      break;

    case 0x06:  // readmsg
      if (inBoxData[inBoxTail].msgLen) {
        LOGI("READMSG @%d", inBoxTail);
        apiCmdResp(cmd | RES_SUCCESS, seq, inBoxData[inBoxTail].msgObj, inBoxData[inBoxTail].msgLen);
      } else {
        // respond "error, no messages"
        tBuf[0] = 0xff;
        apiCmdResp(cmd | RES_FAILURE, seq, tBuf, 1);
      }
      break;

    case 0x07:  // nextmsg
      if (inBoxData[inBoxTail].msgLen) {  // length > 0 indicates message exists
        // respond with number of messages LEFT, in our case zero
        tBuf[0] = tBuf[1] = 0;
        LOGI("NEXTMSG @%d", inBoxTail);
        inBoxData[inBoxTail].msgLen = 0;  // "delete" message by setting length to zero
        inBoxTail = (inBoxTail+1) % INBOX_SIZE;
        LOGI("MSGQLEN=%d", inBoxMsgs());
        apiCmdResp(cmd | RES_SUCCESS, seq, tBuf, 2);
        if (!inBoxMsgs())
          apiSetFlag(0);
      } else {
        // respond 00 00 "error, nothing to delete"
        tBuf[0] = tBuf[1] = 0;
        apiCmdResp(cmd | RES_FAILURE, seq, tBuf, 2);
      }
      break;

    case 0x0b:  // rst_gid
      LOGI("Reset GIDs");
      memset(myGIDSet, 0, sizeof(myGIDSet));
      myGIDNum = 0;
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x10:  // set_app
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x1b:  // UNDOCUMENTED? "SET LED ACTIVE"
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x21:  // set_geo
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    case 0x2c:  // UNDOC - set emergency message?
      apiCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;

    default:
      LOGW("!!! UNHANDLED CMD %02x", cmd);
  }

  return true;
}
