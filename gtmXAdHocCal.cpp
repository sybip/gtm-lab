//
// GTM LAB - goTenna Mesh protocol playground
// ------------------------------------------
// Copyright 2021-2023 by https://github.com/sybip (gpg 0x8295E0C0)
// Released under MIT license (see LICENSE file for full details)
//
// AD-HOC FREQUENCY OFFSET CALIBRATION for GTM LAB
// -----------------------------------------------
// (opportunistically using other nodes' signals)
//
// On startup, listen for a number of packets or up to a number of seconds,
//  average the frequency error of the packets and use the value as our
//  frequency offset freqCorr (for RX **and TX**) until next reboot
// (this is complementary to the AFC, which affects RX only)
// See also: freqCorr and function setChan() in gtmRadio.cpp
//
// To avoid polluting the other nodes' own calibration processes,
//  **relaying is disabled** during calibration to reduce TX activity
//

#include "gtmRadio.h"
#include "gtmConfig.h"
#include "gtmXAdHocCal.h"

#define TAG "GTXCAL"

// variables for adhoc calibration feature
unsigned long calibStarted = 0;  // millis
bool calibRunning = false;
uint8_t calibSamples = 24;
uint16_t calibSeconds = 30;
int8_t calibRegTemp = 0;  // snapshot of temperature at calibration time


// Start an adhoc calibration, optionally specifying the timeout in seconds
void adCalibStart(uint16_t nSamples, uint16_t timeout)
{
  relaying = false;
  calibSamples = nSamples;
  calibSeconds = timeout;
  calibStarted = millis();
  calibRunning = true;
  LOGI("Adhoc calibration started for %d sec", calibSeconds);
}


// Check on a running calibration
bool adCalibCheck()
{
  if (!calibRunning)
    return false;

  if ((feiHistLen >= calibSamples) || (millis()-calibStarted) > 1000*calibSeconds) {
    calibRegTemp = getRadioTemp();  // set once and never changed
    fcorrRegTemp = calibRegTemp;  // set every time we update freqCorr
    if (feiHistLen > 0) {
      LOGI("Calibration complete with %d/%d samples", feiHistLen, calibSamples);
    } else {
      LOGI("Calibration timed out");
    }

    int16_t feiAvg = feiTrimMean(calibSamples);
    LOGI("Applying CORR=%d (at TEMP=%d) and resetting history", feiAvg, calibRegTemp);
    freqCorr += feiAvg;
    feiHistLen = feiHistPos = 0;

    calibRunning = false;
    // re-enable relaying
    relaying = DFLT_RELAY;
  }
  return calibRunning;
}
