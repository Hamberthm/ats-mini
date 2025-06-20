#include "Common.h"
#include "Themes.h"
#include "NimBLEDevice.h"
#include "NuSerial.hpp"

//
// Get current connection status
// (-1 - not connected, 0 - disabled, 1 - connected)
//
int8_t getBleStatus()
{
  if(!NimBLEDevice::isInitialized()) return 0;
  return NimBLEDevice::getServer()->getConnectedCount() > 0 ? 1 : -1;
}

//
// Stop BLE hardware
//
void bleStop()
{
  if(!NimBLEDevice::isInitialized()) return;
  NuSerial.stop();
  // Full deinit
  NimBLEDevice::deinit(true);
}

void bleInit(uint8_t bleMode)
{
  bleStop();

  if(bleMode == BLE_OFF) return;

  NimBLEDevice::init(RECEIVER_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0); // N12, N9, N6, N3, N0, P3, P6, P9
  NimBLEDevice::getAdvertising()->setName(RECEIVER_NAME);
  NuSerial.start();
}

int bleDoCommand(uint8_t bleMode)
{
  if(bleMode == BLE_OFF) return 0;

  if (NuSerial.isConnected()) {
    if (NuSerial.available()) {
      char bleChar = NuSerial.read();
      Serial.print(bleChar);
    }
  }
  return 0;
}
