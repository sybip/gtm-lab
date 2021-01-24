#ifndef GTMCONFIG_H
#define GTMCONFIG_H

//
// USER CONFIGURABLE OPTIONS
// -------------------------
// These are make-or-break settings, so choose carefully
//

// ISM_REGION: 1=US/CA, 2=EU, 4=AU/NZ, 6=TW/JP
#define ISM_REGION 1

// BOARD_TYPE: 1=TBEAM-1.x, 2=WRL-15006
#define BOARD_TYPE 1

// Logging verbosity, please use sparingly:
// - DEBUG may distort protocol timing causing some message loss
// - VERBOSE **will** break the protocol timings and prevent
//       any message reception beyond individual packets
#define VERBOSITY ESP_LOG_INFO

// END of user options

#endif // GTMCONFIG_H
