//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright (c) 2021 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//

// THE HARDCORE PLAYGROUND
// -----------------------
//  do you even lift? :)

// Message class IDs
#define MSG_CLASS_P2P   0
#define MSG_CLASS_GROUP 1
#define MSG_CLASS_SHOUT 2
#define MSG_CLASS_EMERG 3

// GTA Message Body TLVs
#define MSGB_TLV_TYPE 0x01    // Message type, a %d string of a number(!)
#define MSGB_TLV_NICK 0x03    // Message sender nickname
#define MSGB_TLV_TEXT 0x04    // Message body text
#define MSGB_TLV_GDST 0x05    // Destination GID in group messages
#define MSGB_TLV_LCTN 0x06    // Location object
#define MSGB_TLV_PUBK 0xfc    // Public key object


// Assembles and sends a "shout" message with the supplied string as message body
// (thanks to https://gitlab.com/almurphy for the Arduino implementation)
int testShoutTx(char * msgBody)
{
  uint32_t time32 = now();
  uint8_t mData[256];  // message buffer (for a small message)
  uint8_t mPos = 0;
  uint8_t blobPos = 0;

  // Message class and App ID
  mData[mPos++] = MSG_CLASS_SHOUT;
  mData[mPos++] = 0x3f;  // goTenna AppID MSB
  mData[mPos++] = 0xff;  // goTenna AppID LSB

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
    LOGW("Message too long, truncated");
    msgSize = 220;
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

  // fire up the TX
  txStart();

  // send the message (this will also generate and send the SYNC)
  txSendMsg(mData, mPos, 2, 2);

  // finally return to RX mode
  resetState();  // return to RX mode with a clean slate

  LOGI("TX COMPLETE", mPos);
}


// Generate a text of random size and send it as a "shout" message
int testMessage()
{
  size_t msgLen = random(220);

  char mBody[256];

  sprintf(mBody, "L=%03d:", msgLen);
  for (int i=6; i<msgLen; i++) {
    mBody[i] = 0x30 + (i % 10);
  }
  mBody[msgLen] = 0;  // C string termination

  testShoutTx(mBody);
}


// Console input handler
// executed for each line received on the serial console
// - if first character is '!', treat the line as a command
// - otherwise, transmit it in the body of a "shout" message
int conExec()
{
  char hexBuf[3] = {0};
  uint8_t wReg = 0;
  uint8_t wVal = 0;
  uint8_t xChan = 0;

  if (conBuf[0] != '!') {
    // take text from conBuf and shout it
    testShoutTx(conBuf);
    return(0);
  }

  // implement a (very basic) command console
  // available commands:
  //  !m0 = set SLEEP mode
  //  !m1 = set STANDBY mode
  //  !mr = set RX mode
  //  !mt = set TX mode
  //    (all !m commands will also pause chan scanning)
  //  !rr = dump ALL registers
  //  !ri = dump ISR registers
  //  !rf = dump entire FIFO content
  //      (use !r00 to read one FIFO byte)
  //  !rXX = read register XX (in HEX)
  //  !wXXYY = write to register XX value YY
  //      (XX and YY in HEX)
  //  !cDD = change channel (DD in decimal)
  //  !h0 = control chan scanning pause
  //  !h1 = control chan scanning resume
  //  !ta = transmit a random ACK packet
  //  !tm = transmit a random shout message
  //
  if (conBuf[1] == 'm') {
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
      if (xChan >= CHANNELS) {
        LOGW("Channel out of range (0-%d)", (CHANNELS-1));
      } else {
        LOGI("Set Channel: %d", xChan);
        setChan(xChan);
      }

  } else if (conBuf[1] == 'd') {
    if (conBuf[2] == 'd') {
      memcpy(hexBuf, conBuf+3, 2);
      txPackDelay = strtoul(hexBuf, NULL, 10) & 0xff;
      LOGI("PACKDELAY now %dms", txPackDelay);
    }

  } else if (conBuf[1] == 'r') {
    // VIEW
    if (conBuf[2] == 'r') {
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

    } else {
      memcpy(hexBuf, conBuf+2, 2);
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
    if (conBuf[2] == 'a') {
      // TEST/ACK - send an ACK packet for a randomly generated HashID

      txStart();    // fire up transmitter
      txSendAck(random(65535), 1, 3, 2);
      resetState(); // return to RX mode

    } else if (conBuf[2] == 'm') {
      // TEST/MSG - send a random message
      testMessage();
    }

  } else {
    LOGW("UNKNOWN COMMAND");
  }
}
