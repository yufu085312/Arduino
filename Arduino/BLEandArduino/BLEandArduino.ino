#include <bluefruit.h>

#define VOLTAGE_PIN A0 
#define THRESHOLD_VOLTAGE 13.0 

BLEService engineService = BLEService(0x181A);  // カスタムサービス
BLECharacteristic engineStateCharacteristic = BLECharacteristic(0x2A58);  // カスタムキャラクタリスティック

void setup()
{
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("CarBeacon");

  engineService.begin();
  engineStateCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  engineStateCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  engineStateCharacteristic.begin();

  Serial.begin(115200);
}

void loop()
{
  float voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0);

  if (voltage > THRESHOLD_VOLTAGE) {
    engineStateCharacteristic.write8(true);
  } else {
    engineStateCharacteristic.write8(false);
  }

  delay(1000);
}
