//
// GTM BLE - goTenna Mesh Bluetooth LE emulator
// --------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// Based on https://gitlab.com/almurphy/ESP32GTM
// Copyright 2020 by Alec Murphy, MIT licensed
//

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  // DO NOT EDIT THIS LINE

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2904.h>

#include "gtmBLE.h"

#ifdef ARDUINO_ARCH_ESP32
// It is STRONGLY RECOMMENDED to run BLE loop on a separate core, if possible
#define BLE_DUAL_CORE 1
#define BLE_CORE 0  // Core 0 unused on ESP32 Arduino
#endif   // ARDUINO_ARCH_ESP32

#define TAG "GTMBLE"

#include "esp_log.h"

#define LOG_ ESP_LOG_LEVEL_LOCAL
#define HEXD(tag, buf, len) ESP_LOG_BUFFER_HEXDUMP(tag, buf, len, ESP_LOG_DEBUG)
#define HEXV(tag, buf, len) ESP_LOG_BUFFER_HEXDUMP(tag, buf, len, ESP_LOG_VERBOSE)

// Logging macros, common to ESP and non-ESP
#define LOGE( format, ... ) LOG_(ESP_LOG_ERROR,   TAG, format, ##__VA_ARGS__)
#define LOGW( format, ... ) LOG_(ESP_LOG_WARN,    TAG, format, ##__VA_ARGS__)
#define LOGI( format, ... ) LOG_(ESP_LOG_INFO,    TAG, format, ##__VA_ARGS__)
#define LOGD( format, ... ) LOG_(ESP_LOG_DEBUG,   TAG, format, ##__VA_ARGS__)
#define LOGV( format, ... ) LOG_(ESP_LOG_VERBOSE, TAG, format, ##__VA_ARGS__)

// #define DEVINFO_FULLBRANDING  // enable goTenna spoofing in DEVINFO service (not required)

#define GTMODEL GTM  // GTM = goTenna Mesh or GT1 = goTenna v1

// API result codes (using upper 2 bits) - from gtmAPI.h
#ifndef RES_SUCCESS
#define RES_SUCCESS 0x40
#define RES_FAILURE 0xc0
#endif

BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLEService *dService = NULL;

BLECharacteristic * pCharaS2C;
BLECharacteristic * pCharaC2S;
BLECharacteristic * pCharaST;

bool bleDeviceActive = false;
bool bleDeviceConnected = false;
bool bleOldDevConnected = false;
unsigned long tLastConnChg = 0;

uint32_t cntConnects = 0;   // connects and reconnects
uint32_t cntCommands = 0;   // number of commands processed
uint32_t cntRSuccess = 0;   // number of commands succeeded
uint32_t cntRFailure = 0;   // number of commands failed
uint32_t cntRNeither = 0;   // number of commands returned neither success nor failure
uint32_t cntCRC16BAD = 0;   // CRC16 errors in C2S packets

#define KEEP_ALIVE_ST 10000  // how often (ms) to send ST notifies even if ST not changed
unsigned long tLastST = millis();

// max size of BLE TX packets
#define BLE_TXSIZE 20

// user hook for command handling
bool (* gtmCmdExec)(uint8_t, uint8_t, uint8_t *, size_t) = builtinCmdExec;

void gtmBLELoopCommon(bool quick);

#ifdef BLE_DUAL_CORE
TaskHandle_t TaskBLE;

void TaskBLEcode( void * pvParameters )
{
  LOGI("BLE coretask STARTED on core %d", xPortGetCoreID());

  for(;;) {
    gtmBLELoopCommon(false);  // take all the time that we need
    // loop nice and smooth, to keep watchdog happy
    //delay(10);    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif  // BLE_DUAL_CORE

uint8_t bleServerAddr[6] = { 0 };  // BLE MAC address of our thing
uint8_t bleClientAddr[6] = { 0 };  // BLE MAC address of client

// hard-coded GTM sysinfo block - cloned from gtmAPI.cpp
// Many GTM apps will poll the sysinfo service to check battery status and other
//  system health data, and may freak out if we don't return a valid response
// Ideally, we should construct this block with actual and up-to-date information,
//  however in a minimal case we can just send this block to keep the ball in play
//
uint8_t sysInfoBlock[] = { 
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

BLEAdvertisementData advData = BLEAdvertisementData();
BLEAdvertising *pAdvertising;

unsigned long tStart = 0;

uint8_t txBuf[256];
uint8_t txBufPos = 0;
uint16_t txBufCRC = 0;
bool txBufRdy = false;  // when buffer complete and ready for processing

uint8_t rxBuf[256];
uint8_t rxBufPos = 0;
uint16_t rxBufCRC = 0;
bool rxBufRdy = false;  // when buffer complete and ready for processing

uint8_t gtmStCur = 0;
bool gtmStNew = false;  // status changed, push to client

bool escape = false;    // used statefully by gtmRX()


// CRC16-XMODEM - used in BLE PDUs
uint16_t CRC16X_add(uint8_t b, uint16_t crc)
{
  crc ^= ((uint16_t)b << 8);

  for (int ix = 8; ix; ix--) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;  // (polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}


// almost-copy of ESP32GTM gtmNotifyCallbackRX()
void gtmRX(uint8_t * rxData, uint8_t rxSize)
{
    uint8_t head = 0;
    uint8_t tail = 0;

    // 10 02 is the STX word; initialize our buffer
    if ((rxData[0] == 0x10) &&
        (rxData[1] == 0x02)) {
      rxBufPos = 0;
      rxBufCRC = 0;
      head = 2;
    }

    // 10 03 is the ETX word
    if ((rxData[rxSize-2] == 0x10) &&
        (rxData[rxSize-1] == 0x03)) {
      tail = 2;  // skip ETX
    }

    // skip STX (head) and ETX (tail) if present
    for (int ix=head; ix<rxSize-tail; ix++) {
      // unescape 0x1010 to 0x10
      if (rxData[ix] == 0x10) {
        if (escape) {
          rxBuf[rxBufPos++] = 0x10;
          escape = false;
        } else
          escape = true;
      } else
        rxBuf[rxBufPos++] = rxData[ix];
    }

    // ETX present means packet complete and ready to process
    if (tail) {
      // rxBuf contains UNESCAPED data and 2 bytes CRC16X

      // Calculate CRC for data in rxBuf
      for (int ix=0; ix<rxBufPos-2; ix++) {
        rxBufCRC = CRC16X_add(rxBuf[ix], rxBufCRC);
      }
          
      // Check calculated CRC against expected value
      uint16_t wantCRC = (rxBuf[rxBufPos-2] << 8) + rxBuf[rxBufPos-1];
      if (rxBufCRC != wantCRC) {
          cntCRC16BAD++;
          LOGW("CRC ERROR (WANT=%04x, HAVE=%04x)", wantCRC, rxBufCRC);
          ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, rxBufPos, ESP_LOG_DEBUG);
          return;
      }

    } else {
      // NOT TAIL -> more data chunks to come
      return;
    }

    // Roll back 2 bytes to strip CRC
    rxBufPos -= 2;

    LOGV("RECV C2S:");
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, rxBufPos, ESP_LOG_VERBOSE);

    // indicate buffer is ready
    rxBufRdy = true;
}


uint8_t fragIX = 0;


// Fragment the content of txBuf into BLE_TXSIZE chunks and send ONE
//   fragment to client, storing the curent fragment ID in global fragIX
// This is the preferred method because it only blocks for the duration of
//   one packet, but is still a pain
//
// WARNING uses indicate() which may block up to 150ms
bool gtmSendS2CFrag()
{
  tStart = millis();

  LOGD("SEND S2CF %d:", fragIX);
  ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf, txBufPos, ESP_LOG_VERBOSE);

  // Send buffer to TX handle
  // Fragment TX buffer to BLE_TXSIZE
  if (fragIX+BLE_TXSIZE <= txBufPos) {
    // full packet
    pCharaS2C->setValue(txBuf+fragIX, BLE_TXSIZE);
  } else {
    // last packet not full
    pCharaS2C->setValue(txBuf+fragIX, (txBufPos % BLE_TXSIZE));
  }

  pCharaS2C->indicate();  // may BLOCK up to 150ms

  LOGD("S2CF done, %dms", millis()-tStart);
  fragIX += BLE_TXSIZE;
  if (fragIX < txBufPos)
    return false;   // more frags

  return true;  // no more frags
}


// Fragment the content of txBuf into BLE_TXSIZE chunks and send it
//   to client, all in one call
// This is the OBSOLETE method because it blocks for the duration of several
//   packets. See gtmSendS2CFrag() below for a preferred method
//
// WARNING uses indicate() which may block up to n*150ms
void gtmSendS2C()
{
  tStart = millis();

  LOGV("SEND S2C:");

  // basically just call the new function repeatedly
  while (!gtmSendS2CFrag());

  LOGD("S2C done, %dms", millis()-tStart);
}


void gtmSendSts(uint8_t sts)
{
  tStart = millis();

  LOGI("NOTIFYST: %02x", sts);

  pCharaST->setValue(&sts, 1);
  pCharaST->notify();  // nonblock
}


void gtmBLEFlag(uint8_t st)
{
  gtmStCur = st;
  gtmStNew = true;
}


// identical copy from ESP32GTM
/* 
 * Append byte to BLE TX buffer, calculating CRC in parallel
 * - escaping of special character 0x10 also performed here
 */
void gtmTxBufAppend(uint8_t oneByte)
{
    txBuf[txBufPos++] = oneByte;
    txBufCRC = CRC16X_add(oneByte, txBufCRC);

    // escape 0x10 (but do NOT update CRC)
    if (oneByte == 0x10)
      txBuf[txBufPos++] = oneByte;
}


// based on ESP32GTM gtmSendCMD()
void gtmPostS2C(uint8_t cmd, uint8_t seq, uint8_t* data, size_t len)
{
  // Start with STX word
  txBuf[0] = 0x10;
  txBuf[1] = 0x02;
  txBufPos = 2;
  txBufCRC = 0;

  // Append command byte
  gtmTxBufAppend(cmd);

  // Append sequence byte (must match the one sent by client)
  gtmTxBufAppend(seq);

  // Append payload, byte by byte
  for (int ix = 0; ix < len; ix++) {
    gtmTxBufAppend(data[ix]);
  }

  // Append CRC
  txBuf[txBufPos++] = (0xff & (txBufCRC>>8));
  txBuf[txBufPos++] = (0xff & txBufCRC);

  // Append ETX word
  txBuf[txBufPos++] = 0x10;
  txBuf[txBufPos++] = 0x03;
  
  // Ready to transmit
  txBufRdy = true;
}


// Send a command response, keep it simple
void gtmCmdResp(uint8_t cmd, uint8_t seq, uint8_t* data, size_t len)
{
  LOGI("RES=%02x SEQ=%02x LEN=%d", cmd, seq, len);
  // peek at the response and update our counters
  switch(cmd & 0xc0) {
    case RES_SUCCESS:
      cntRSuccess++;
      break;
    case RES_FAILURE:
      cntRFailure++;
      break;
    default:
      cntRNeither++;
      break;
  }
  gtmPostS2C(cmd, seq, data, len);
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
      // param defined in: bt/host/bluedroid/api/include/api/esp_gatts_api.h
      //
      //    struct gatts_connect_evt_param {
      //        uint16_t conn_id;               /*!< Connection id */
      //        uint8_t link_role;              /*!< Link role : master role = 0  ; slave role = 1*/
      //        esp_bd_addr_t remote_bda;       /*!< Remote bluetooth device address */
      //        esp_gatt_conn_params_t conn_params; /*!< current Connection parameters */
      //    } connect;                          /*!< Gatt server callback param of ESP_GATTS_CONNECT_EVT */
      //
      ESP_LOG_BUFFER_HEXDUMP("CLIMAC: ", param->connect.remote_bda, 6, ESP_LOG_INFO);
      LOGI("Init params: interval=%d, latency=%d, timeout=%d", 
            param->connect.conn_params.interval,
            param->connect.conn_params.latency,
            param->connect.conn_params.timeout);

      // Update connection parameters can be called only after connection has been established
      // updateConnParams(esp_bd_addr_t remote_bda, uint16_t minInterval, uint16_t maxInterval, uint16_t latency, uint16_t timeout) {
      pServer->updateConnParams(param->connect.remote_bda, 0x10, 0x20, 0, 0x40);
      memcpy(bleClientAddr, param->connect.remote_bda, 6);
      bleDeviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      memset(bleClientAddr, 0xff, 6);
      bleDeviceConnected = false;
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        LOGV("BLE[RX]: %d", (rxValue.length()));
        gtmRX((uint8_t *) rxValue.data(), rxValue.length());
        // will accumulate into rxBuf and set rxBufRdy when done
      }
    }
};


// In order to be able to compile for multiple GTM models with different UUIDs,
//   we use a preprocessor trick:
// 1) define GTMODEL as GTM or GT1
// 2) in the code, concatenate GTMODEL with fixed part of UUID name to obtain
//   desired name, e.g GTM + _DEVINFO_SERVICE = GTM_DEVINFO_SERVICE
//
#define _CONCAT(a, b) a##b
#define CONCAT(a, b) _CONCAT(a, b)


// Initialize and start BLE service
// Typically called from Arduino setup(), but can also be called after BLE was
//   turned off via gtmBLEStop() to turn it back on
void gtmBLEInit(char * devName)
{
  // Create the BLE Device
  BLEDevice::init(devName);  // BLE device name
  
  // get MAC address (needed below)
  BLEAddress myAddr = BLEDevice::getAddress();
  memcpy(bleServerAddr, myAddr.getNative(), 6);

  ESP_LOG_BUFFER_HEXDUMP("BLEMAC: ", bleServerAddr, 6, ESP_LOG_DEBUG);

  // for addr type RANDOM STATIC, top 2 bits of MSB must be 11
  // (ref Bluetooth spec 4.0 sec 10.8.1)
  bleServerAddr[0] |= 0xc0;
  ESP_LOG_BUFFER_HEXDUMP("RNDMAC: ", bleServerAddr, 6, ESP_LOG_DEBUG);

  if (!BLEDevice::getInitialized()) {
    LOGI("BLE device FAILED INIT");
    return;
  }

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create Device Information Service, which may be needed for detection
  dService = pServer->createService(CONCAT(GTMODEL, _DEVINFO_SERVICE));
#if DEVINFO_FULLBRANDING
  dService->createCharacteristic(CONCAT(GTMODEL, _DEVINFO_MANUF_NAME),
                                 BLECharacteristic::PROPERTY_READ)->setValue("Gotenna");
  dService->createCharacteristic(CONCAT(GTMODEL, _DEVINFO_MODEL_NUMBER),
                                 BLECharacteristic::PROPERTY_READ)->setValue("goTenna Mesh");
  dService->createCharacteristic(CONCAT(GTMODEL, _DEVINFO_HARDWARE_VER),
                                 BLECharacteristic::PROPERTY_READ)->setValue("00");
  dService->createCharacteristic(CONCAT(GTMODEL, _DEVINFO_FIRMWARE_VER),
                                 BLECharacteristic::PROPERTY_READ)->setValue("1.1.8");
#endif
  dService->start();

  // Create the BLE Service
  pService = pServer->createService(CONCAT(GTMODEL, _SERVICE_UUID));

  // Create ST(atus) BLE Characteristic
  pCharaST = pService->createCharacteristic(CONCAT(GTMODEL, _CHARACT_UUID_ST),
                         BLECharacteristic::PROPERTY_NOTIFY | 
                         BLECharacteristic::PROPERTY_READ);
  pCharaST->addDescriptor(new BLE2902());
  pCharaST->addDescriptor(new BLE2904());
  // pCharaST->setCallbacks(new MyCallbacks());  // not needed

  // C2S (app to GTM = "TX" on app side)
  pCharaC2S = pService->createCharacteristic(CONCAT(GTMODEL, _CHARACT_UUID_C2S),
                          BLECharacteristic::PROPERTY_WRITE | 
                          BLECharacteristic::PROPERTY_READ | 
                          BLECharacteristic::PROPERTY_WRITE_NR);
  // this is our receiving characteristic - set callbacks here
  pCharaC2S->setCallbacks(new MyCallbacks());

  // Create S2C ("RX" on app side) BLE Characteristic
  pCharaS2C = pService->createCharacteristic(CONCAT(GTMODEL, _CHARACT_UUID_S2C),
                          BLECharacteristic::PROPERTY_INDICATE);
  pCharaS2C->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // tweak the advert to make it look more gtm
  advData.setFlags(0x05);
  advData.setCompleteServices(BLEUUID(CONCAT(GTMODEL, _SERVICE_UUID)));
  
  pAdvertising = BLEDevice::getAdvertising();

  // set type random
  pAdvertising->setDeviceAddress(bleServerAddr, BLE_ADDR_TYPE_RANDOM);

  // set advertising data
  pAdvertising->setAdvertisementData(advData);

  // Start advertising
  BLEDevice::startAdvertising();
  bleDeviceActive = true;

#ifdef BLE_DUAL_CORE
  xTaskCreatePinnedToCore(
      TaskBLEcode,
      "TaskBLE",
      10000,
      NULL,
      1,
      &TaskBLE,
      BLE_CORE);
#endif
  LOGI("BLE services STARTED");
}


// Stop BLE service and emulation
void gtmBLEStop()
{
#ifdef BLE_DUAL_CORE
  // kill loop task
  if (TaskBLE != NULL) {
    vTaskDelete(TaskBLE);
    LOGI("BLE coretask DELETED");  
  }
#endif  // BLE_DUAL_CORE
  BLEDevice::deinit();
  bleDeviceActive = false;
  LOGI("BLE services STOPPED");  
}


// Query status of BLE service: disabled, enabled, connected?
uint8_t gtmBLEStatus()
{
  if (! bleDeviceActive) {
    return BLE_SERVICE_DISABLED;
  }

  if (bleDeviceConnected) {
    return BLE_SERVICE_CONNECTED;
  }

  return BLE_SERVICE_ACCEPTING;
}


// Call from Arduino loop to perform BLE periodic tasks
// can be called in two ways:
//  - quick=true if time is critical and must return quickly
//  - quick=false if a delay of up to 300ms is acceptable
// 
void gtmBLELoop(bool quick)
{
  // If running the BLE loop on its own core, there's no need to hook into
  // the Arduino main loop function anymore, so this function becomes a no-op
  // just in case some legacy code keeps calling it from main loop
#ifndef BLE_DUAL_CORE
  gtmBLELoopCommon(quick);
#endif
}


void gtmBLELoopCommon(bool quick)
{
  // disconnecting
  if (!bleDeviceConnected && bleOldDevConnected) {
    LOGI("BLE Disconnected");

    if (bleDeviceActive) {
      // resume advertising
      tLastConnChg = millis();
      BLEDevice::startAdvertising();
      LOGI("BLE Advertising");
    }
    // cleanup tasks that need to be done once, at the end of client session

    bleOldDevConnected = bleDeviceConnected;
  }

  // connecting
  if (bleDeviceConnected && !bleOldDevConnected) {
    LOGI("BLE Connecting");
    bleOldDevConnected = bleDeviceConnected;
    tLastConnChg = millis();
    cntConnects++;
    // init tasks that need to be done once, at the start of client session
  }

  if (gtmBLEStatus() != BLE_SERVICE_CONNECTED) {
    // nothing else to do here
    return;
  }

  // any command in the BLE buffer? execute it now
  if (rxBufRdy) {
    uint8_t cmd = rxBuf[0];
    uint8_t seq = rxBuf[1];

    cntCommands++;
    LOGI("CMD=%02x SEQ=%02x LEN=%d", cmd, seq, rxBufPos-2);
    gtmCmdExec(cmd, seq, rxBuf+2, rxBufPos-2);

    // release buffer for the next command
    rxBufPos = 0;
    rxBufCRC = 0;
    rxBufRdy = false;
  }

  // any status change? send to client now
  // (nonblocking, can send during time restrictions)
  if ((millis()-tLastST > KEEP_ALIVE_ST) || gtmStNew) {
    gtmSendSts(gtmStCur);
    gtmStNew = false;
    tLastST = millis();
  }

  // blocking operations, only if there's no rush
  if (!quick) {
    // any s2c object queued? send to client now
    if (txBufRdy) {
/*
      // obsolete
      gtmSendS2C();
      // release buffer for the next command
      txBufPos = 0;
      txBufCRC = 0;
      txBufRdy = false;
*/
      // send segments individually to reduce blocking
      if (gtmSendS2CFrag()) {  // will return true if last segment
        txBufPos = 0;
        txBufCRC = 0;
        txBufRdy = false;
        fragIX = 0;
      }
    }
  }
}


// This is an absolutely minimal command handler that should trick most
//  apps to believe that they're talking to an actual GTM device
// Most cases we only need to return a positive response to the commands,
//  but in a few cases we need to return some actual credible data
// These cases are: SYSINFO, SENDMSG, READMSG and (optionally) NEXTMSG
//
// NOTE TO DEVELOPERS: as a matter of protocol, a command handler must always
//  return a response, by calling gtmCmdResp() at the end. See below.
//
bool builtinCmdExec(uint8_t cmd, uint8_t seq, uint8_t * buf, size_t len)
{
  uint8_t tBuf[4] = { 0 };

  switch (cmd) {
    case 0x03:  // sendmsg - response is "33 01 ff"
      tBuf[0] = 0x33;
      tBuf[1] = 0x01;
      tBuf[2] = 0xff;   // undocumented
      gtmCmdResp(cmd | RES_SUCCESS, seq, tBuf, 3);
      break;

    case 0x04:  // sysinfo
      // send hard-coded sysinfo block
      gtmCmdResp(cmd | RES_SUCCESS, seq, sysInfoBlock, 28);
      break;

    case 0x06:  // readmsg
      // respond "error, no messages"
      tBuf[0] = 0xff;   // undocumented
      gtmCmdResp(cmd | RES_FAILURE, seq, tBuf, 1);
      break;

    case 0x07:  // nextmsg
      // respond 00 00 "error, no mext message"
      tBuf[0] = tBuf[1] = 0;
      gtmCmdResp(cmd | RES_FAILURE, seq, tBuf, 2);
      break;

    default:
      gtmCmdResp(cmd | RES_SUCCESS, seq, NULL, 0);
      break;
  }

  return true;
}
