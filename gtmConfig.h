#ifndef GTMCONFIG_H
#define GTMCONFIG_H

//
// USER CONFIGURABLE OPTIONS
// -------------------------
// These are make-or-break settings, so choose carefully
//

// BOARD_TYPE: 1=TBEAM-1.x, 2=WRL-15006
#define BOARD_TYPE 1

// Default REGION: 1=US/CA, 2=EU, 4=AU/NZ, 8=TW/JP
#define ISM_REGION 1

// Default TX power (dBm, 0-20) - can be changed at runtime
#define DFLT_POWER 0   // 0dBm = 1mW, OK for benchtop tests

// Default relay mode - can be changed at runtime
#define DFLT_RELAY false   // relaying disabled at startup

// Default App ID - can be changed at runtime
#define DFLT_APPID 0x7357  // use testnet by default

// Logging verbosity, please use sparingly:
// - DEBUG may distort protocol timing causing some message loss
// - VERBOSE **will** break the protocol timings and prevent
//       any message reception beyond individual packets
// Verbosity can be changed at runtime
#define VERBOSITY ESP_LOG_INFO

// END of user options

#define MAX_TX_POWER 20  // 20 dBm = 100mW

#endif // GTMCONFIG_H
