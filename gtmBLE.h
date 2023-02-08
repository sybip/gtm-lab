//
// GTM BLE - goTenna Mesh Bluetooth LE emulator
// --------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// Based on https://gitlab.com/almurphy/ESP32GTM
// Copyright 2020 by Alec Murphy, MIT licensed
//

#ifndef GTMBLE_H
#define GTMBLE_H

#include <Arduino.h>

// BLE MAC address, read at startup
extern uint8_t bleServerAddr[6];

// returned by gtmBLEStatus
#define BLE_SERVICE_DISABLED 0
#define BLE_SERVICE_ACCEPTING 1
#define BLE_SERVICE_CONNECTED 2

// default device name
#define BLE_DEV_NAME "gtmThing"

// GTM service UUID
#define GTM_SERVICE_UUID    "1276aaee-df5e-11e6-bf01-fe55135034f3"
#define GTM_CHARACT_UUID_RX "1276b20b-df5e-11e6-bf01-fe55135034f3" // server to client
#define GTM_CHARACT_UUID_TX "1276b20a-df5e-11e6-bf01-fe55135034f3" // client to server
#define GTM_CHARACT_UUID_ST "12762b18-df5e-11e6-bf01-fe55135034f3"

#define GTM_CHARACT_UUID_S2C GTM_CHARACT_UUID_RX
#define GTM_CHARACT_UUID_C2S GTM_CHARACT_UUID_TX

#define GTM_DEVINFO_SERVICE         "0000180a-0000-1000-8000-00805f9b34fb"
#define GTM_DEVINFO_MANUF_NAME      "00002a29-0000-1000-8000-00805f9b34fb"
#define GTM_DEVINFO_MODEL_NUMBER    "00002a24-0000-1000-8000-00805f9b34fb"
#define GTM_DEVINFO_HARDWARE_VER    "00002a27-0000-1000-8000-00805f9b34fb"
#define GTM_DEVINFO_FIRMWARE_VER    "00002a26-0000-1000-8000-00805f9b34fb"

// GT1 service UUID
// WARNING - all GT1 data is based on PURE GUESSWORK;
// the author has never seen a v1 goTenna in real life
#define GT1_SERVICE_UUID    "f0abaaee-ebfa-f96f-28da-076c35a521db"
#define GT1_CHARACT_UUID_RX "f0abb20b-ebfa-f96f-28da-076c35a521db" // server to client
#define GT1_CHARACT_UUID_TX "f0abb20a-ebfa-f96f-28da-076c35a521db" // client to server
#define GT1_CHARACT_UUID_ST "f0ab2b18-ebfa-f96f-28da-076c35a521db"

#define GT1_CHARACT_UUID_S2C GT1_CHARACT_UUID_RX
#define GT1_CHARACT_UUID_C2S GT1_CHARACT_UUID_TX

//#define GT1_DEVINFO_SERVICE         "0000180a-0000-1000-8000-00805f9b34fb"
#define GT1_DEVINFO_SERVICE         "00001801-0000-1000-8000-00805f9b34fb"
#define GT1_DEVINFO_FIRMWARE_VER    "00002a26-0000-1000-8000-00805f9b34fb"

// hard-coded "last resort" GTM sysinfo block
extern uint8_t sysInfoBlock[];

extern unsigned long tLastConnChg;   // millis time of last connection change

// counters
extern uint32_t cntConnects;   // connects and reconnects
extern uint32_t cntCommands;   // number of commands processed
extern uint32_t cntRSuccess;   // number of commands successed
extern uint32_t cntRFailure;   // number of commands failed
extern uint32_t cntRNeither;   // number of commands returned neither success nor failure
extern uint32_t cntCRC16BAD;   // CRC16 errors in C2S packets

// Initialize and start BLE services
// Typically called from Arduino setup(), but can also be called after BLE was
//   turned off via gtmBLEStop() to turn it back on
// If multi-core, also starts the periodic BLE loop on second core
void gtmBLEInit(char * devName = BLE_DEV_NAME);

// Stop and disable BLE services
// If multi-core, also kills the periodic BLE loop on second core
// (call gtmBLEInit() to re-enable)
void gtmBLEStop();

// Call from Arduino loop to perform BLE periodic tasks
// can be called in two ways:
//  - quick=true if time is critical and must return quickly
//  - quick=false if a delay of up to 300ms is acceptable
// In multi-core operation, this function is a NO-OP, since periodic
//  tasks are performed from the BLE loop (TaskBLE) on second core
void gtmBLELoop(bool quick = false);

// Query status of BLE service: disabled, enabled, connected?
uint8_t gtmBLEStatus();

// Send a 1-byte status flag to client via BLE ST characteristic
void gtmBLEFlag(uint8_t st);

// Send a command response back to client via BLE S2C characteristic
void gtmCmdResp(uint8_t cmd, uint8_t seq, uint8_t* data, size_t len);

// command handler function - must call gtmCmdResp before return
extern bool (* gtmCmdExec)(uint8_t, uint8_t, uint8_t *, size_t);

// builtin handler
bool builtinCmdExec(uint8_t cmd, uint8_t seq, uint8_t * buf, size_t len);

#endif // GTMBLE_H
