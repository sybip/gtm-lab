//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

#include "gtmConfig.h"
#include "gtmNode.h"
#include "logging.h"

#define TAG "GTMLIB"

// message hash ringbuffer size, for duplicate detection
// there will be 4 of these: ACK RX/TX and Msg RX/TX
#define HASH_BUF_LEN 64

// short-term buffers for queuing outbound messages 
//   while waiting for a clear channel to transmit
// buffer sizes are unscientific broad estimates
#define MSG_QUEUE_SIZE 6
#define ACK_QUEUE_SIZE 6

struct msgDesc {
  uint8_t iniTTL;
  uint8_t curTTL;
  uint16_t hashID;
  unsigned long tStamp;
  uint16_t msgLen;
  uint8_t msgObj[500];  // FIXME explain
};

struct ackDesc {
  uint8_t iniTTL;
  uint8_t curTTL;
  uint16_t hashID;
  unsigned long tStamp;
  uint8_t hops;
};

// Msg hash ringbuffers
uint16_t rxAckBuf[HASH_BUF_LEN] = { 0 };
uint16_t txAckBuf[HASH_BUF_LEN] = { 0 };
uint16_t rxMsgBuf[HASH_BUF_LEN] = { 0 };
uint16_t txMsgBuf[HASH_BUF_LEN] = { 0 };
uint8_t rxAckPos = 0;
uint8_t txAckPos = 0;
uint8_t rxMsgPos = 0;
uint8_t txMsgPos = 0;

msgDesc msgQueue[MSG_QUEUE_SIZE];
ackDesc ackQueue[ACK_QUEUE_SIZE];

uint8_t msgQHead = 0, msgQTail = 0;
uint8_t ackQHead = 0, ackQTail = 0;

bool relaying = DFLT_RELAY;  // enable mesh relay function
bool noDeDup = false;  // disable duplicate filtering in RX
bool ezOutput = DFLT_EZOUTPUT;  // enable simple hexdump output of RX/TX events

uint16_t txInertia = 0;   // INERTIA - millis to wait before TX
uint16_t txInerMAX = 600; // INERTIA - maximum value

// PACKET COUNTERS
// ACK packets received
// uint32_t cntRxPktACKTot = 0;  // total
uint32_t cntRxPktACKUni = 0;  // unique
// ACK packets transmitted
//uint32_t cntTxPktACKTot = 0;  // total
uint32_t cntTxPktACKRel = 0;  // relayed
// DATA objects received
uint32_t cntRxDataObjTot = 0;  // total
uint32_t cntRxDataObjUni = 0;  // unique
// DATA objects transmitted
uint32_t cntTxDataObjTot = 0;  // total
uint32_t cntTxDataObjRel = 0;  // relayed

// FIXME 20230508 temporary workarounds section
#define TTRK_FIXME
bool gtmlabRxTask();
bool gtmlabBusy();
int txSendAckOne(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL);
int txSendMsgOne(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL);
// end of FIXME section

// user-definable event handler callbacks
// MSG rx handler
bool (* onRxMSG)(uint8_t *, uint16_t, uint8_t, uint8_t, uint8_t, uint16_t) = builtinRxMSG;
// ACK rx handler
bool (* onRxACK)(uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t) = builtinRxACK;
// Rx error handler
bool (* onRxERR)(uint16_t) = builtinRxERR;
// MSG Tx handler
bool (* onTxMSG)(uint16_t) = builtinTxMsgOK;
// ACK Tx handler
bool (* onTxACK)(uint16_t) = builtinTxAckOK;

void echoCount(uint16_t hashID, bool isACK, bool isOwn = false);

// GTH16 hash algorithm (based on park-miller LCG)
// WARNING: 64-bit multiplication used
uint16_t gtAlgoH16(uint8_t* str, size_t len) 
{
    uint32_t seed = 0xaa;
    uint32_t mult = 48271;
    uint32_t incr = 1;
    uint32_t modulus = (1<<31) -1; // 0x7FFFFFFF;

    uint32_t h = 0;
    uint64_t x = seed;
    for (int i = 0; i<len; i++) {
        x = (((x + str[i]) * mult + incr) & 0xFFFFFFFF) % modulus;
        h ^= x;
    }

    // Derive 16-bit value from 32-bit hash by XORing its two halves
    return ((h & 0xFFFF0000) >> 16) ^ (h & 0xFFFF);
}


// calculates message hash using GTH16 algo
uint16_t msgHash16(uint8_t* mBuf)
{
  // position of HEAD element (depends on message class)
  uint8_t headPos = 5;

  if (mBuf[0] == MSG_CLASS_GROUP) {
    headPos = 12;   // destination address
  } else if (mBuf[0] == MSG_CLASS_P2P) {
    headPos = 15;   // destination + element 04
  }

  // FIXME - do a sanity check on TLV header? (FB 10)

  return gtAlgoH16(mBuf+headPos, 16);
}


// Look up a hash in one of the hash ringbuffers
bool inRingBuf(uint16_t needle, uint16_t *haystack)
{
  for (int i = 0; i < HASH_BUF_LEN; i++) {
    if (haystack[i] == needle)
      return true;
  }
  return false;
}


// Look up a hash in MSG output queue
bool inMsgQueue(uint16_t hash16)
{
  for (int i = 0; i < MSG_QUEUE_SIZE; i++) {
    if (msgQueue[i].curTTL && (msgQueue[i].hashID == hash16))
      return true;
  }
  return false;
}


// Look up a hash in ACK output queue
bool inAckQueue(uint16_t hash16)
{
  for (int i = 0; i < ACK_QUEUE_SIZE; i++) {
    if (ackQueue[i].curTTL && (ackQueue[i].hashID == hash16))
      return true;
  }
  return false;
}


// Queue a message for sending (when channel is clear)
bool txEnQueueMSG(uint8_t * msgObj, uint16_t msgLen, uint8_t iniTTL, uint8_t curTTL)
{
  if (msgQueue[msgQHead].curTTL>0) {
    // buffer is full, retry later
    return false;
  }
  msgQueue[msgQHead].iniTTL = iniTTL;
  msgQueue[msgQHead].curTTL = curTTL;
  msgQueue[msgQHead].tStamp = millis();
  msgQueue[msgQHead].msgLen = msgLen;
  memcpy(msgQueue[msgQHead].msgObj, msgObj, msgLen);
  msgQHead = (msgQHead+1) % MSG_QUEUE_SIZE;

  return true;
}


// Queue an ACK for sending (when channel is clear)
bool txEnQueueACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL)
{
  if (ackQueue[ackQHead].curTTL>0) {
    // buffer is full, retry later
    return false;
  }
  ackQueue[ackQHead].iniTTL = iniTTL;
  ackQueue[ackQHead].curTTL = curTTL;
  ackQueue[ackQHead].tStamp = millis();
  ackQueue[ackQHead].hashID = hashID;
  ackQueue[ackQHead].hops = hops;
  ackQHead = (ackQHead+1) % ACK_QUEUE_SIZE;

  return true;
}


// Check (e.g from Arduino loop) if tx queue contains any objects
//   (which would be tx-ed on the next call to gtmTxTask)
bool txQueueEmpty()
{
  if (msgQueue[msgQTail].curTTL > 0)
    return false;

  if (ackQueue[ackQTail].curTTL > 0)
    return false;

  return true;
}


// FIXME make function to check if TX backed off
extern unsigned long txBackOff; // Tx retry backoff

// called from main loop to check for TX tasks and execute the first in queue
// TIMING: <230ms when log=W and output disabled (due to serial bottleneck)
bool gtmlabTxTask()
{
  bool txHold = false;  // hold the transmission of current object (INERTIA)
  bool txLocal = true;  // the message is originated locally
  bool txRes = false;  // TX Result

  if (txBackOff > millis())
  //  txHold = true;    // Hold TX if backoff engaged
    // since the final LBT unjam check was moved to LBT function, at this point
    //  we can just return, because nothing else happens during a TX Backoff
    return false;

  unsigned long tStart = millis();

  // Quick notes on INERTIA feature:
  // When relaying a received object, all nodes on the network would normally
  // start the relay process at the same time, increasing the risk of collisions
  // such as reported in issue #3
  // The INERTIA feature mitigates this by introducing a random delay before
  // the transmission of a relayed object (but not locally originated objects,
  // which do not suffer from this issue)
  // It is similar, but not identical to the concept of random backoff; the
  // difference is the delay here is applied proactively not reactively

  // Anything queued for sending?
  if (msgQueue[msgQTail].curTTL > 0) {
    if (msgQueue[msgQTail].iniTTL != msgQueue[msgQTail].curTTL) {
      // curTTL == iniTTL indicates a locally originated object (not relayed)
      // only for relayed objects, use INERTIA to delay TX start
      txLocal = false;
      if (millis() - msgQueue[msgQTail].tStamp < txInertia) {
        txHold = true;
      }
    }

    if (! txHold) {
      tStart = millis();

      // save a copy of hash ID to survive dequeue
      uint16_t tHash = msgHash16(msgQueue[msgQTail].msgObj);

      txRes = txSendMsgOne(msgQueue[msgQTail].msgObj, msgQueue[msgQTail].msgLen,
                           msgQueue[msgQTail].iniTTL, msgQueue[msgQTail].curTTL);

      if (txRes) {
        // save to ringbuffer
        txMsgBuf[txMsgPos++] = tHash;
        txMsgPos %= HASH_BUF_LEN;

        // advance the queue
        memset(&(msgQueue[msgQTail]), 0, sizeof(struct msgDesc));
        msgQTail = (msgQTail+1) % MSG_QUEUE_SIZE;

        if(txLocal && txRes) {
          echoCount(tHash, false, true);  // count an own MSG
          unsigned long t0 = millis();  // track time spent in handler
          if (! onTxMSG(tHash)) {
            LOGD("onTxMSG handler failed");
          }
#ifndef TTRK_FIXME
          TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
#endif
        }
        return true;
      }  //  if txRes
    }  // if (! txHold)

  } else if (ackQueue[ackQTail].curTTL > 0) {
    if (ackQueue[ackQTail].iniTTL != ackQueue[ackQTail].curTTL) {
      // curTTL == iniTTL indicates a locally originated object (not relayed)
      txLocal = false;
      if (millis() - ackQueue[ackQTail].tStamp < txInertia) {
        txHold = true;
      }
    }

    if (! txHold) {
      tStart = millis();

      // save a copy of hash ID to survive dequeue
      uint16_t tHash = ackQueue[ackQTail].hashID;

      txRes = txSendAckOne(ackQueue[ackQTail].hashID, ackQueue[ackQTail].hops,
                ackQueue[ackQTail].iniTTL, ackQueue[ackQTail].curTTL);

      if (txRes) {
        // save to ringbuffer
        txAckBuf[txAckPos++] = tHash;
        txAckPos %= HASH_BUF_LEN;

        // advance the queue
        memset(&(ackQueue[ackQTail]), 0, sizeof(struct ackDesc));
        ackQTail = (ackQTail+1) % ACK_QUEUE_SIZE;

        if(txLocal) {
          echoCount(tHash, true, true);  // count an own ACK
          unsigned long t0 = millis();  // track time spent in handler
          if (! onTxACK(tHash)) {
            LOGD("onTxACK handler failed");
          }
#ifndef TTRK_FIXME
          TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
#endif
        }
        return true;
      }  // if txRes
    }  // if !txHold
  }

  return false;  // nothing was sent
}


// node-specific counters are reset here
void nodeResetCounters()
{
  cntRxDataObjTot = cntRxDataObjUni = cntRxPktACKUni = 0;
  cntTxDataObjTot = cntTxDataObjRel = cntTxPktACKRel = 0;
}


// called from gtmRadio to report an RX error
void handleRxERR(uint16_t error)
{
  // invoke external handler
  unsigned long t0 = millis();  // track time spent in handler
  if ((* onRxERR) != nullptr)
    onRxERR(error);
#ifndef TTRK_FIXME
  TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
#endif
}


// called from rxPacket to submit an incoming ACK object for processing
bool processRxACK(uint16_t msgH16, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  bool isUnique = true;  // first time we see this object

  echoCount(msgH16, true);  // count an ACK

  if (inRingBuf(msgH16, rxAckBuf)) {
    isUnique = false;
    LOGI("ACK SEEN BEFORE");
  } else {
    cntRxPktACKUni++;
    rxAckBuf[rxAckPos++] = msgH16;
    rxAckPos %= HASH_BUF_LEN;

    // what's this ACK for?
    if (inRingBuf(msgH16, txMsgBuf)) {
      LOGI("ACK for TXd MSG");
    } else if (inRingBuf(msgH16, rxMsgBuf)) {
      LOGI("ACK for RXd MSG");
    } else {
      LOGI("ACK without MSG");
    }

    // Is this an echo of an ACK that we sent?
    if (inRingBuf(msgH16, txAckBuf)) {
      LOGI("ACK SENT BEFORE");
    // have we already queued a copy for sending?
    } else if (inAckQueue(msgH16)) {
      LOGI("ACK IN TX QUEUE");
    } else {
      // if first seen, and curTTL>1, it's a relayable ACK
      if (curTTL > 1) {
        ////////// RELAYING //////////
        LOGI("RELAYACK (%s)", (relaying ? "ON!":"OFF"));
        if (relaying) {
          // decrement curTTL and enqueue for TX
          // FIXME catch error
          txEnQueueACK(msgH16, hops, iniTTL, (curTTL - 1));
          cntTxPktACKRel++;
        }
      }
    }
  }

  if (isUnique || noDeDup) {
    // new ACK received - call external handler if defined
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxACK) != nullptr) {
      if (! onRxACK(msgH16, hops, iniTTL, curTTL, uRSSI, FEI)) {
        LOGD("onRxACK handler failed");
      }
    }
#ifndef TTRK_FIXME
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
#endif
  }

  return true;
}


// called from rxPacket to submit an incoming MSG object for processing
bool processRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  bool isUnique = true;  // first time we see this object
  uint16_t msgH16 = msgHash16(mBuf);

/* // FIXME, also datastart may be incorrect start point (if it discounts the SYNC)
  LOGI("RX complete: len=%d, hash=0x%04x, time=%dms, RSSI=-%d",
       mLen, msgH16, (millis()-dataStart), uRSSI);
*/
  LOGI("RX complete: len=%d, hash=0x%04x, time=?, RSSI=-%d",
       mLen, msgH16, uRSSI);
  echoCount(msgH16, false);  // count an MSG
  cntRxDataObjTot++;

  if (inRingBuf(msgH16, rxMsgBuf)) {
    LOGI("MSG SEEN BEFORE");
    isUnique = false;
  } else {
    cntRxDataObjUni++;
    rxMsgBuf[rxMsgPos++] = msgH16;
    rxMsgPos %= HASH_BUF_LEN;

    // Is this an echo of a message that we sent?
    if (inRingBuf(msgH16, txMsgBuf)) {
      LOGI("MSG SENT BEFORE");
    // have we already queued a copy for sending?
    } else if (inMsgQueue(msgH16)) {
      LOGI("MSG IN TX QUEUE");
    } else {
      // if first seen, and curTTL>1, it's a relayable message
      if (curTTL>1) {
        ////////// RELAYING //////////
        LOGI("RELAYMSG (%s)", (relaying ? "ON!":"OFF"));
        if (relaying) {
          // decrement curTTL and enqueue for TX
          // FIXME catch error
          txEnQueueMSG(mBuf, mLen, iniTTL, (curTTL - 1));
          cntTxDataObjRel++;
        }
      }
    }
  }

  if (isUnique || noDeDup) {
    // New message received - call external handler if defined
    unsigned long t0 = millis();  // track time spent in handler
    if ((* onRxMSG) != nullptr) {
      if (! onRxMSG(mBuf, mLen, iniTTL, curTTL, uRSSI, FEI)) {
        LOGD("onRxMSG handler failed");
      }
    }
#ifndef TTRK_FIXME
    TTRK[ttGetPeriod()].timeEvt += (millis() - t0);
#endif
  }

  return true;
}


// simple builtin MSG handler - hexlified output
// (uRSSI because the RSSI is unsigned)
bool builtinRxMSG(uint8_t * mBuf, uint16_t mLen, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  if (ezOutput) {  // easy output enabled
    // output in standardized format (inittl, curttl, uRSSI, | ,hexmsg)
    printf("RX_MSG:");
    printf("%02x%02x%02x|", iniTTL, curTTL, uRSSI);
    for (int i=0; i<mLen; i++)
      printf("%02x", mBuf[i]);
    printf("\n");
  }
  return true;
}


// simple builtin ACK handler - hexlified output
bool builtinRxACK(uint16_t hashID, uint8_t hops, uint8_t iniTTL, uint8_t curTTL, uint8_t uRSSI, uint16_t FEI)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("RX_ACK:%04x%02x%02x%02x\n", hashID, ((hops & 0x0f)<<4) | (iniTTL & 0x0f), curTTL, uRSSI);
  return true;  
}


// simple builtin RX Error handler
bool builtinRxERR(uint16_t error)
{
  // do nothing
  return true;
}


// simple builtin TX MSG succcess handler - output is hex HashID of message
bool builtinTxMsgOK(uint16_t hashID)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("TX_MSG:%04x\n", hashID);
  return true;
}


// simple builtin TX ACK succcess handler - output is hex HashID
bool builtinTxAckOK(uint16_t hashID)
{
  if (ezOutput)  // easy output enabled
    // output in standardized format
    printf("TX_ACK:%04x\n", hashID);
  return true;
}


// Call from Arduino loop to perform RX and TX tasks
void gtmlabLoop()
{
  gtmlabRxTask();

  // if (scanning && !recvData) {   // no operation in progress
  if (!gtmlabBusy()) {   // no operation in progress
    // Check and perform one TX task
    if (gtmlabTxTask()) {
      // something was transmitted
    }
  }
}


////////// Some additional tools & experiments below //////////

// ECHO COUNTER
// For each object TXd or RXd, count the number of copies we received back
// Two important applications:
// - roughly estimate the size of the reachable network
// - puddingproof the relaying efficiency of our nodes (when starting up
//   a new LAB relay, do we CONSISTENTLY see an extra echo? compare with
//   GTM relays)
// Call echoCount(hashID, isACK), the number of echoes will be logged at INFO

#define ECHOSIMTRACK 8  // max echoes to track simultaneously, of each type
struct echoDesc {
  unsigned long tStamp;
  uint16_t hashID;
  uint8_t count;
};

// one dataset for MSGs, one for ACKs
echoDesc Echoes[ECHOSIMTRACK * 2] = {0};
uint8_t echoHead[2] = {0};

// make some of these variables externally available
uint8_t echoCur = 0;
uint16_t echoAvg = 0;  // x100
char echoFmt[33] = {0};  // formatted


void echoCount(uint16_t hashID, bool isACK, bool isOwn)
{
  uint8_t hx = 0;  // which dataset, MSG or ACK
  uint8_t i = 0;   // index into arrays

  if (isACK)
    hx = 1;

  echoDesc * e = & Echoes[hx * ECHOSIMTRACK];

  bool found = false;
  for (i = 0; i < ECHOSIMTRACK; i++) {
    if (e[i].hashID == hashID) {
      found = true;
      break;
    }
  }
  if (!found) {
    i = echoHead[hx];
    e[i].hashID = hashID;
    e[i].count = 0;
    e[i].tStamp = millis();
    echoHead[hx] = (echoHead[hx] + 1) % ECHOSIMTRACK;
  }

  if (!isOwn) {
    // only increment for non-own packets
    e[i].count++;
  }

  int cnt = 0;
  int sum = 0;

  // Format the last ECHOSIMTRACK message echo counts, new-to-old
  // FIXME? is the reversed arrow-of-time a problem?
  char formatted[33] = {0};
  int pos = 0;
  for (int j = 0; j < ECHOSIMTRACK; j++) {
    // sum for average
    if ((i != j) && (e[j].count >0)) {
      sum += e[j].count;
      cnt ++;
    }

    // format for output
    pos += sprintf(formatted + pos, "%d ", e[(ECHOSIMTRACK+i-j) % ECHOSIMTRACK].count);
  }
  if (cnt) {
    if (! isACK) {
      // copy to extern variables
      echoAvg = 100 * sum / cnt;
      echoCur = e[i].count;
      strncpy(echoFmt, formatted, 32);
    }
    LOGI("ECHO(%s:%04x)=%d t=%d AVG=%02.2f [ %s]", isACK ? "ACK":"MSG", hashID,
         e[i].count, millis() - e[i].tStamp, 1.0 * sum / cnt, formatted);
  }
  return;
}

////
