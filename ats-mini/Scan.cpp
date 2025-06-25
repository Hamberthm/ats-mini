#include "Common.h"
#include "Utils.h"
#include "Menu.h"
#include <map>

#define SCAN_TIME   100 // Msecs between tuning and reading RSSI
#define SCAN_POINTS 200 // Number of frequencies to scan
#define SCAN_SEAMLESS 41 // Max points of frequencies to save on Seamless Scan map (currently, 41 is total points and is MINIMUM)

#define SCAN_OFF    0   // Scanner off, no data
#define SCAN_RUN    1   // Scanner running
#define SCAN_DONE   2   // Scanner done, valid data in scanData[]

static struct
{
  uint8_t rssi;
  uint8_t snr;
} scanData[SCAN_POINTS];

static uint32_t scanTime = millis();
static uint8_t  scanStatus = SCAN_OFF;

static uint16_t scanStartFreq;
static uint16_t scanStep;
static uint16_t scanCount;
static uint8_t  scanMinRSSI;
static uint8_t  scanMaxRSSI;
static uint8_t  scanMinSNR;
static uint8_t  scanMaxSNR;

static inline uint8_t min(uint8_t a, uint8_t b) { return(a<b? a:b); }
static inline uint8_t max(uint8_t a, uint8_t b) { return(a>b? a:b); }

static std::map<uint16_t, float> scanMap;
static const char *scanSeamlessLastBand = getCurrentBand()->bandName;
static uint8_t scanSeamlessBypass = 0;

static float scanSeamless(uint16_t freq)
{
  if(scanMap.contains(freq))
  {
    return((scanMap[freq] - scanMinRSSI) / (float)(scanMaxRSSI - scanMinRSSI + 1));
  }

  if (strcmp(scanSeamlessLastBand, getCurrentBand()->bandName)) //do not scan during a band change
  {
    scanSeamlessLastBand = getCurrentBand()->bandName;
    scanSeamlessBypass = SCAN_SEAMLESS * 2; //hold by 2 screen refreshes at least
  }

  if (scanSeamlessBypass)
  {
    scanSeamlessBypass--;
    return 0.0;
  }

  // Save current frequency
  uint16_t curFreq = rx.getFrequency();

  rx.setFrequency(freq);

  if (currentMode == AM)
    delay(30); //AM needs an extra 30 ms on top to correctly measure

  // Measure RSSI/SNR values
  rx.getCurrentReceivedSignalQuality();
  scanMap[freq] = rx.getCurrentRSSI();

  // Measure range of values
  scanMinRSSI = scanMaxRSSI = 0;
  for (auto it = scanMap.begin(); it != scanMap.end(); it++)
  {
  scanMinRSSI = min(it->second, scanMinRSSI);
  scanMaxRSSI = max(it->second, scanMaxRSSI);
  }

  // Restore current frequency
  rx.setFrequency(curFreq);

  if (scanMap.size() > SCAN_SEAMLESS) //keep map below 45 elements (about screen wide)
  {
    if ((freq - scanMap.begin()->first) < (std::prev(scanMap.end())->first) - freq) //erase the farthest end of the map, measured from the current freq, that is guaranteed to be off screen
      scanMap.erase(std::prev(scanMap.end())); //end() gives us a past-the-end theoretical iterator, we need to go back 1 iteration
    else
      scanMap.erase(scanMap.begin());
  }

  return((scanMap[freq] - scanMinRSSI) / (float)(scanMaxRSSI - scanMinRSSI + 1));
}

float scanGetRSSI(uint16_t freq)
{
    if(true) //put switch for Seamless Scan here
  {
   return scanSeamless(freq);
  }

  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || (freq>=scanStartFreq+scanStep*scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].rssi;
  return((result - scanMinRSSI) / (float)(scanMaxRSSI - scanMinRSSI + 1));
}

float scanGetSNR(uint16_t freq)
{
  // Input frequency must be in range of existing data
  if((scanStatus!=SCAN_DONE) || (freq<scanStartFreq) || (freq>=scanStartFreq+scanStep*scanCount))
    return(0.0);

  uint8_t result = scanData[(freq - scanStartFreq) / scanStep].snr;
  return((result - scanMinSNR) / (float)(scanMaxSNR - scanMinSNR + 1));
}

static void scanInit(uint16_t centerFreq, uint16_t step)
{
  scanStep    = step;
  scanCount   = 0;
  scanMinRSSI = 255;
  scanMaxRSSI = 0;
  scanMinSNR  = 255;
  scanMaxSNR  = 0;
  scanStatus  = SCAN_RUN;
  scanTime    = millis();

  const Band *band = getCurrentBand();
  int freq = scanStep * (centerFreq / scanStep - SCAN_POINTS / 2);

  // Adjust to band boundaries
  if(freq + scanStep * (SCAN_POINTS - 1) > band->maximumFreq)
    freq = band->maximumFreq - scanStep * (SCAN_POINTS - 1);
  if(freq < band->minimumFreq)
    freq = band->minimumFreq;
  scanStartFreq = freq;

  // Clear scan data
  memset(scanData, 0, sizeof(scanData));
}

static bool scanTickTime()
{
  // Scan must be on
  if((scanStatus!=SCAN_RUN) || (scanCount>=SCAN_POINTS)) return(false);

  // Wait for the right time
  if(millis() - scanTime < SCAN_TIME) return(true);

  // This is our current frequency to scan
  uint16_t freq = scanStartFreq + scanStep * scanCount;

  // If frequency not yet set, set it and wait until next call to measure
  if(rx.getFrequency() != freq)
  {
    rx.setFrequency(freq);
    scanTime = millis();
    return(true);
  }

  // Measure RSSI/SNR values
  rx.getCurrentReceivedSignalQuality();
  scanData[scanCount].rssi = rx.getCurrentRSSI();
  scanData[scanCount].snr  = rx.getCurrentSNR();

  // Measure range of values
  scanMinRSSI = min(scanData[scanCount].rssi, scanMinRSSI);
  scanMaxRSSI = max(scanData[scanCount].rssi, scanMaxRSSI);
  scanMinSNR  = min(scanData[scanCount].snr, scanMinSNR);
  scanMaxSNR  = max(scanData[scanCount].snr, scanMaxSNR);

  // Next frequency to scan
  freq += scanStep;

  // Set next frequency to scan or expire scan
  if((++scanCount >= SCAN_POINTS) || !isFreqInBand(getCurrentBand(), freq))
    scanStatus = SCAN_DONE;
  else
    rx.setFrequency(freq);

  // Save last scan time
  scanTime = millis();

  // Return current scan status
  return(scanStatus==SCAN_RUN);
}

//
// Run entire scan once
//
void scanRun(uint16_t centerFreq, uint16_t step)
{
  // Save current frequency
  uint16_t curFreq = rx.getFrequency();
  // Scan the whole range
  for(scanInit(centerFreq, step) ; scanTickTime() ; delay(SCAN_TIME));
  // Restore current frequency
  rx.setFrequency(curFreq);
}
