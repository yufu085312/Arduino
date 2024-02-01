#include <bluefruit.h>

#define VOLTAGE_PIN A0 //アナログピンのA0A0を定義している
#define THRESHOLD_VOLTAGE 13.0 //エンジンのON OFを判断するための閾値

// iBeacon UUID, Major, and Minor values
// Advertisementパケットデータ
uint8_t beaconData[] = {
  // AdvertisementのFlagsセクション
  0x02, // セクションの長さ
  0x01, // FlagsのADタイプ
  0x06, // Flagsの値、LE General Discoverable Mode、BR/EDR Not Supported

  // Manufacturer Specific Dataセクション
  0x1A, // セクションの長さ
  0xFF, // Manufacturer Specific DataのADタイプ
  0x4C, 0x00, // AppleのCompany Identifier Code

  0x02, // iBeaconのAdvertisementインジケータ
  0x15, // UUID、Major、Minor、およびTx Powerの合計バイト数
  // iBeacon proximity UUID
  0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25,
  0x00, 0x00, // iBeaconのMajor値、グループやカテゴリ 
  0x00, 0x00, // iBeaconのMajor値、特定のiBeaconデバイス 
  0xC3  // キャリブレーションされたTx Powerの2の補数
};

void setup()
{
  Bluefruit.begin(); //初期化
  Bluefruit.setTxPower(4); //送信電力の設定
  Bluefruit.setName("CarStatusBeacon"); //BLEデバイスの名前

  Serial.begin(115200); //シリアル通信

  // Bluefruitの初期化
  if (Bluefruit.begin())
  {
    Serial.println("Bluefruit初期化成功!");
  }
  else
  {
    Serial.println("Bluefruit初期化失敗。");
    while(1); // ここで無限ループに入り、他のコードが実行されないようにします。
  }

  Bluefruit.setTxPower(4); //送信電力の設定
  Bluefruit.setName("CarStatusBeacon"); //BLEデバイスの名前
}

void loop()
{
  float voltage = analogRead(VOLTAGE_PIN) * (5.0 / 1023.0);

  if (voltage > THRESHOLD_VOLTAGE) {
    beaconData[25] = 0x00;
    beaconData[26] = 0x01; // Set major to 1
  } else {
    beaconData[25] = 0x00;
    beaconData[26] = 0x00; // Set major to 0
  }

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, beaconData, sizeof(beaconData));
  Bluefruit.Advertising.start(0);

  delay(1000);
}
