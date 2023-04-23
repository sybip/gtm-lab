// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// AD-HOC FREQUENCY OFFSET CALIBRATION for GTM LAB
//

#ifndef GTMXADHOCCAL_H
#define GTMXADHOCCAL_H

#define ADHOC_CALIBRATION

extern unsigned long calibStarted;  // millis
extern bool calibRunning;

// Start an adhoc calibration, optionally specifying the timeout in seconds
// Will temporarily TURN OFF RELAYING to reduce uncalibrated transmissions
void adCalibStart(uint16_t nSamples = 24, uint16_t timeout = 90);

// Poll a running adhoc calibration operation; if a completion condition
//  has been reached, set freqCorr offset to the average FEI of collected
//  samples and resume relaying if originally enabled
// All values are in native FSTEP units (~61Hz)
bool adCalibCheck();

#endif  // GTMXADHOCCAL_H
