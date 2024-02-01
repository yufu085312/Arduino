//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : LTE-M
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : LPWA King demo
//
//     Leaf configuration
//       (1) AC04 LPWA King
//       (2) AI01 4-Sensors
//       (3) AI04 LCD
//       (4) AP01 AVR MCU
//       (5) AZ01 USB
//
//    (c) 2019 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2019/9/30  First release
//=====================================================================
//注意：本スケッチは1時間あたり150Kbyte程度の送受信が発生します。通信容量にご注意ください
//=====================================================================
//use libraries
//Adafruit LIS3DH
//https://github.com/adafruit/Adafruit_LIS3DH
//※  Adafruit_LIS3DH.h
//    uint8_t readRegister8(uint8_t reg);
//    void writeRegister8(uint8_t reg, uint8_t value);
//    をpublic:に移動する
//Adafruit Unified Sensor Driver
//https://github.com/adafruit/Adafruit_Sensor
//SmartEverything ST HTS221 Humidity Sensor
//https://github.com/ameltech/sme-hts221-library
//ClosedCube Arduino Library for ClosedCube OPT3001
//https://github.com/closedcube/ClosedCube_OPT3001_Arduino
//ST7032 - Arduino LiquidCrystal compatible library
//https://github.com/tomozh/arduino_ST7032
//MsTimer2 Library
//https://github.com/PaulStoffregen/MsTimer2
//=====================================================================
//=====================================================================
//※LTE-Mリーフへの1回の送信受信データが64byteを超える場合、SoftwareSerial.hで定義
//  されている受信バッファサイズを変更する必要があります。
//  本スケッチでは256byteに変更することをお勧めします。
//  受信バッファサイズの変更は、SoftwareSerial.hを直接下記の通り変更します。
//     #define _SS_MAX_RX_BUFF 64 → 256
//=====================================================================
//#define _SS_MAX_RX_BUFF 128

//=====================================================================
// include
//=====================================================================
#include <Arduino.h> 
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>    
#include <Adafruit_Sensor.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include <ST7032.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

//=====================================================================
// 送信間隔　(秒）
//=====================================================================
#define SEND_INTERVAL 60
//=====================================================================

//=====================================================================
// デバック＆リーフ設定
//=====================================================================
#define LCD 1      // LCD Leaf use:1  Grove leaf use:0
#define USB_EN 0   // USB Leaf use: 1 ( USB Leafからログを出力する場合) 

//=====================================================================
// IFTTT Webhook
// Webhookのイベント名とKEY番号を記載
//=====================================================================
const char event_name[] = "LTE-M Leaf";  // Webhookのイベント名
const char key[] = "";   // WebhookのKEY番号
//=====================================================================
// LTE 接続設定
// 契約したSIMカードの情報を入力する
//=====================================================================
const char apn[] = "soracom.io";         //APN名
const char user_name[] = "sora";         //ユーザー名
const char password[] = "sora";          //パスワード

//=====================================================================
// port define
//=====================================================================
// --------------------------------------------
// PD port
//     digital 0: PD0 = USB UART TX
//     digital 1: PD1 = USB UART RX
//     digital 2: PD2 = Reseved (BLE.SENS INT Port)
//     digital 3: PD3 = RI
//     digital 4: PD4 = PWR_ON_N# 
//     digital 5: PD5 = RST_N#
//     digital 6: PD6 = Reseved(BLE DISCN)
//     digital 7: PD7 = Reseved(BLSLP#)
// --------------------------------------------
#define PCTX      0
#define PCRX      1
#define INT0_N    2
#define RI        3
#define PWR_ON_N  4
#define RST_N     5
#define RSV_D6    6
#define RSV_D7    7
// --------------------------------------------
// PB port
//     digital 8: PB0 = UART0_RX  
//     digital 9: PB1 = UART0_TX  
//     digital 10:PB2 = SS#       /* not use */
//     digital 11:PB3 = MOSI      /* not use */
//     digital 12:PB4 = DTR
//     digital 13:PB5 = DSR
//                PB6 = XTAL1
//                PB7 = XTAL2
//---------------------------------------------
#define UART1_TX  8       /* HL7800 UART */
#define UART1_RX  9       /* HL7800 UART */
#define WAKEUP    10
#define RTS       11
#define DTR       12
#define DSR       13
// --------------------------------------------
// PC port
//     digital 14/ Analog0: PC0 = Reseved
//     digital 15/ Analog1: PC1 = Reseved (BLE UART TX) 
//     digital 16/ Analog2: PC2 = Reseved (BLE UART RX)  
//     digital 17/ Analog3: PC3 = CTS
//     digital 18/ Analog4: PC4 = SDA
//     digital 19/ Analog5: PC5 = SDL 
//     RESET              : PC6 = RESET#
//-----------------------------------------------
#define RSV_D14 14
#define RSV_D15 15
#define RSV_D16 16
#define CTS     17
#define SDA     18
#define SCL     19
/**********************************************
* define
**********************************************/
//-----------------------------------------------
// loop() interval
// MsTimer2のタイマー割り込み発生間隔(ms)
//-----------------------------------------------
#define LOOP_INTERVAL 1000         // 1000ms interval

/**********************************************
* I2C define
**********************************************/
#define I2C_EXPANDER_ADDR       0x21	// LTE-M Leaf IO EXPANCER I2C ADDR
#define I2C_EXPANDER_ADDR_LCD   0x1A	// LCD Leaf IO EXPANCER I2C ADDR
#define I2C_ADC_ADDR            0x52	// LTE-M Leaf ADC   I2C ADDR
#define LIS2DH_ADDRESS			    0x19	// Senser Leaf accel Senser i2c ADDR(SD0/SA0 pin = VCC) 
#define OPT3001_ADDRESS			    0x45  // Senser Leaf Optical Senser I2C ADDR(ADDR pin = VCC)
#define I2C_GROVE_ADDR			    0x18  // Grove Reaf IO EXPANCER I2C ADDR

#define I2C_RECEIVE_BUF_LENGTH 10
unsigned char i2c_receiveBuf[I2C_RECEIVE_BUF_LENGTH];
unsigned char i2c_receiveLenght;

Adafruit_LIS3DH accel = Adafruit_LIS3DH();

//---------------------------
// OPT3001 : Light
//---------------------------
ClosedCube_OPT3001 light;

#if LCD
  ST7032 lcd;
#endif    

SoftwareSerial SerialHL7800(UART1_RX, UART1_TX); // RX, TX
/**********************************************
* 変数定義
**********************************************/

//---------------------------
// LIS2DH : accelerometer
//---------------------------
float dataX_g, dataY_g, dataZ_g;

//---------------------------
// HTS221 : Temperature/Humidity
//---------------------------
float dataTemp;
float dataHumid;

//---------------------------
// 2点補正用データ
//---------------------------
// 温度補正用データ0
float TL0 = 25.0;     // 4-Sensors温度測定値
float TM0 = 25.0;     // 温度計等測定値
// 温度補正用データ1
float TL1 = 40.0;     // 4-Sensors温度測定値
float TM1 = 40.0;     // 温度計等測定値

// 湿度補正用データ0
float HL0 = 60.0;     // 4-Sensors湿度測定値
float HM0 = 60.0;     // 湿度計等測定値
// 湿度補正用データ1
float HL1 = 80.0;     // 4-Sensors湿度測定値
float HM1 = 80.0;     // 湿度計等測定値

//---------------------------
// OPT3001 : Light
//---------------------------
short dataLight;

//---------------------------
// ADC081C021: Battry Volteage
//---------------------------
float dataBatVolt;
String str;

volatile int sendSts = 0; 
volatile int readSenser = 0; 
volatile unsigned long timercounter = 0; 

volatile int sw_sts_old ; 
volatile int lcd_chg = 0;
volatile int lcd_view = 0; 

//---------------------------
// 文字列
//---------------------------
String getTime = "NOTIME";


/**********************************************
* 割り込み処理初期設定
**********************************************/
//-----------------------------------------------
// external interrupt
// 外部割り込み設定
//-----------------------------------------------
void setupExtInt(){

//  attachInterrupt(0, intExtInt0, FALLING);    // Sw1    INT0# = enabled
//  detachInterrupt(1);                         // sensor INT1# = disabled
}

//-----------------------------------------------
// timer2 interrupt (interval=125ms, int=overflow)
// メインループのタイマー割り込み設定
//-----------------------------------------------
void setupTC2Int(){
  MsTimer2::set(LOOP_INTERVAL, intTimer2);
}

//=====================================================================
// 割り込み処理
// 
//=====================================================================
//=====================================================================
// interrupt
//=====================================================================
//----------------------------------------------
// Timer2 INT
// タイマー割り込み関数
//----------------------------------------------
void intTimer2(){
    timercounter++;
    if (timercounter >= SEND_INTERVAL)
    {
      timercounter = 0;
      readSenser= 1;
    }
    if (timercounter % 2 == 0)
    {
      lcd_chg = 1;
    }
}

//----------------------------------------------
// INT0
// INT0割り込み関数
//----------------------------------------------
void intExtInt0(){
}

//----------------------------------------------
// INT1
// INT1割り込み関数
//----------------------------------------------
void intExtInt1(){
}
/**********************************************
* I2C スレーブデバイスに1バイト書き込む
**********************************************/
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}
/**********************************************
* I2C スレーブデバイスから1バイト読み込む
**********************************************/
unsigned char i2c_read_byte(int device_address, int reg_address){

  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}
/**********************************************
* I2C スレーブデバイスに複数バイト書き込む
**********************************************/
void i2c_write(int device_address, int reg_address, int lengrh, unsigned char* write_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  for (int i = 0; i < lengrh; i++){
    Wire.write(write_byte[i]);
  }
  Wire.endTransmission();
}
/**********************************************
* I2C スレーブデバイスから複数バイト読み込む
**********************************************/
void i2c_read(int device_address, int reg_address, int lengrh, unsigned char* read_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, lengrh);
  for (int i = 0; i < lengrh; i++){
    read_byte[i] = Wire.read();
  }
}
/**********************************************
* I2C 受信バッファクリア
**********************************************/
void clearI2CReadbuf(){
  memcpy(i2c_receiveBuf, 0x00, I2C_RECEIVE_BUF_LENGTH);
}
//=====================================================================
// 各デバイスの初期設定
// 
//=====================================================================
/**********************************************
* sensor
**********************************************/
void setupSensor(){

  //-------------------------------------
  // LIS2DH (accelerometer)
  //-------------------------------------
  accel.begin(LIS2DH_ADDRESS);

  accel.writeRegister8(LIS3DH_REG_TEMPCFG, 0x00);  // Temp senser disable
  accel.writeRegister8(LIS3DH_REG_CTRL1, 0x07);    //X,Y,Z axis = enable
  accel.setDataRate(LIS3DH_DATARATE_1_HZ);         //Data rate = 1Hz
  accel.writeRegister8(LIS3DH_REG_CTRL2, 0x00);
  accel.writeRegister8(LIS3DH_REG_CTRL3, 0x00);    // INT Disable
  accel.writeRegister8(LIS3DH_REG_CTRL4, 0x80);    //BUD = enable, Scale = +/-2g

 
  //-------------------------------------
  // HTS221 (temperature /humidity)
  //-------------------------------------
  smeHumidity.begin();

  //-------------------------------------
  // OPT3001 (light)
  //-------------------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);

  newConfig.RangeNumber = B1100;               // automatic full scale
  newConfig.ConvertionTime = B1;               // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;   // continous conversion
  newConfig.Latch = B0;                        // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if (errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);   //retry
  } 
}

float getBatVolt()
{
  unsigned char resVal;
  float retVal;
  clearI2CReadbuf();
  i2c_read(I2C_ADC_ADDR, 0x00, 2, i2c_receiveBuf);
  resVal = (i2c_receiveBuf[0] << 4 ) | ((i2c_receiveBuf[1] &0xF0) >> 4);
  retVal = (float)((((double)resVal * 3300 * 2) / 256) / 1000);
  
  return retVal;  
}

void getSenserData()
{
    /* LIS2DH */
    accel.read();
    dataX_g = accel.x_g;    //X軸
    dataY_g = accel.y_g;    //Y軸
    dataZ_g = accel.z_g;    //Z軸
    
    /* HTS221 */
    dataTemp = (float)smeHumidity.readTemperature();  //温度
    dataHumid = (float)smeHumidity.readHumidity();    //湿度

    //-------------------------
    // 温度と湿度の2点補正
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // 温度補正
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // 湿度補正

    /* OPT3001 */
    OPT3001 result = light.readResult();
    
    if (result.error == NO_ERROR){
      dataLight = (short)result.lux;                         //照度
    }
    else{
       dataLight = 0;
    }

    /* battry*/
   dataBatVolt = getBatVolt();  
   
#if USB_EN
#if 0
    Serial.println("");
    Serial.println("--- sensor data ---");
    Serial.println("  Tmp[degC]     = " + String(dataTemp));         //小数点2桁まで増やす場合は String(dataTemp,桁数)
    Serial.println("  Hum[%]        = " + String(dataHumid));
    Serial.println("  Lum[lx]       = " + String(dataLight));
    Serial.println("  Bat[V]        = " + String(dataBatVolt));
    Serial.println(" Accel X,Y,Z" + String(dataX_g) + " " + String(dataY_g) + " " + String(dataZ_g));
#endif    
#endif

}
//-----------------------------------------
// sleep sensor
// センサーリーフをスリープさせる
//-----------------------------------------
void sleepSensor(){

	//-----------------------
	// OPT3001 sleep
	//-----------------------
	OPT3001_Config newConfig;
	OPT3001_ErrorCode errorConfig;

	newConfig.ModeOfConversionOperation = B00;
	errorConfig = light.writeConfig(newConfig);
	if (errorConfig != NO_ERROR){

		errorConfig = light.writeConfig(newConfig);
	}

	//-----------------------
	// LIS2DH sleep
	//-----------------------
	accel.setDataRate(LIS3DH_DATARATE_POWERDOWN);

	//-----------------------
	// HTS221 sleep
	//-----------------------
	smeHumidity.deactivate();
}

//-----------------------------------------
// wakeup sensor
// センサーリーフをスリープから復帰させる
//-----------------------------------------
void wakeupSensor(){
	//-----------------------
	// OPT3001 wakeup
	//-----------------------
	OPT3001_Config newConfig;
	OPT3001_ErrorCode errorConfig;

	newConfig.RangeNumber = B1100;               //automatic full scale
	newConfig.ConvertionTime = B1;               //convertion time = 800ms
	newConfig.ModeOfConversionOperation = B11;   //continous conversion
	newConfig.Latch = B1;                        //latch window style

	errorConfig = light.writeConfig(newConfig);
	if (errorConfig != NO_ERROR){

		errorConfig = light.writeConfig(newConfig);   //retry
	}

	//-----------------------
	// LIS2DH wakeup
	//-----------------------
	accel.setDataRate(LIS3DH_DATARATE_1_HZ);

	//-----------------------
	// HTS221 wakeup
	//-----------------------
	smeHumidity.activate();
}

/**********************************************
* Write command
**********************************************/
void writeCommand(String cmd)
{
  // CTSがLOWになるまで待ってRTSをHIGHにする
  while (digitalRead(CTS) == 1){ 
	  returnCommand();
	  delay(1);
  }

  digitalWrite(RTS, HIGH);
  delay(100);
  SerialHL7800.println(cmd);
  delay(100);
  digitalWrite(RTS, LOW);
  delay(100);
}
/**********************************************
* Write command KCNXCFG
**********************************************/
void writeCommand_KCNXCFG(String apname, String uname, String pass)
{
  // CTSがLOWになるまで待ってRTSをHIGHにする
  while (digitalRead(CTS) == 1){ 
    returnCommand();
    delay(1);
  }

  digitalWrite(RTS, HIGH);
  delay(100);
  SerialHL7800.print(F("AT+KCNXCFG=1,\"GPRS\",\""));
  SerialHL7800.print(apname);
  SerialHL7800.print(F("\",\""));
  SerialHL7800.print(uname);
  SerialHL7800.print(F("\",\""));
  SerialHL7800.print(pass);

#if 1
  SerialHL7800.println(F("\"")); //IPV以降のパラメータ省略
#else
  SerialHL7800.println(F("\",\"IPV4\",\"0.0.0.0\",\"0.0.0.0\",\"0.0.0.0\""));
#endif
  delay(100);
  digitalWrite(RTS, LOW);
  delay(100);
}

/**********************************************
* Write command CGDCONT
**********************************************/
void writeCommand_CGDCONT(String apname)
{
  // CTSがLOWになるまで待ってRTSをHIGHにする
  while (digitalRead(CTS) == 1){ 
    returnCommand();
    delay(1);
  }

  digitalWrite(RTS, HIGH);
  delay(100);
  SerialHL7800.print(F("AT+CGDCONT=1,\"IP\",\""));
  SerialHL7800.print(apname);
  SerialHL7800.println(F("\""));
  delay(100);
  digitalWrite(RTS, LOW);
  delay(100);
}
/**********************************************
*  return command
**********************************************/
void returnCommand()
{
  while(SerialHL7800.available() > 0) {       // 受信したデータが存在する
    str = SerialHL7800.readStringUntil('\n'); // 受信データを読み込む 
#if USB_EN
    Serial.println(str); 
#endif    
  }
}

/**********************************************
*  AT commandの応答確認
**********************************************/
int chkRtnMes(String smes)
{
  String readStr;
  String readStrtmp;
  unsigned long sTime;
  unsigned long cTime;
  sTime = millis();
ret2:
  cTime = millis();
  if (cTime >= sTime){
    if ( cTime - sTime > 10000) return 2;
  }else {
      if ((cTime + (4294967295 -  sTime)) > 20000)  return 2;
  }
  
  if (SerialHL7800.available() <= 0) goto ret2;
  readStr="";
  readStrtmp = SerialHL7800.readStringUntil('\n');  
  readStr = readStr + readStrtmp;
  while (!(readStrtmp.endsWith(F("\n")) || readStrtmp.endsWith(F("\r"))))
  {
    readStrtmp = SerialHL7800.readStringUntil('\n'); 
    readStr = readStr + readStrtmp;    
  }
  readStr.replace(F("\r"), F(""));
  readStr.replace(F("\n"), F(""));
  readStr.trim();      
#if USB_EN  
  Serial.println(readStr);
#endif
  if (readStr.startsWith(smes) )
  {    
      return 0;
  }
  else if (readStr.startsWith(F("ERROR")) || readStr.startsWith(F("+CME ERROR")))
  {
      return 1;
  }  
  goto ret2;
  return 3;
}

/**********************************************
*  AT＋KCNX commandの応答確認
**********************************************/
int chkKcnxMes()
{
  String readStr;
  String readStrtmp;
  unsigned long sTime;
  unsigned long cTime;
  sTime = millis();
ret4:
  cTime = millis();
  if (cTime >= sTime){
    if ( cTime - sTime > 10000) return 2;
  }else {
      if ((cTime + (4294967295 -  sTime)) > 10000)  return 2;
  }
  if (SerialHL7800.available() <= 0) goto ret4;
  readStr="";
  readStrtmp = SerialHL7800.readStringUntil('\n');  
  readStr = readStr + readStrtmp;
  while (!(readStrtmp.endsWith(F("\n")) || readStrtmp.endsWith(F("\r"))))
  {
    readStrtmp = SerialHL7800.readStringUntil('\n'); 
    readStr = readStr + readStrtmp;    
  }
  readStr.replace(F("\r"), F(""));
  readStr.replace(F("\n"), F(""));
  readStr.trim();      
#if USB_EN  
  Serial.println(readStr);
#endif
  if (readStr.startsWith(F("+KTCP_IND: 1,1"))) {                                            //TCP session status. 1 session is set up and ready for operation
     return 0;
  }
  if (readStr.startsWith(F("+KCNX_IND: 1,1")) || readStr.startsWith(F("+KCNX_IND: 1,4")))   //PDP connection status 1:Connected 4:Connecting
  {
    goto ret4;
  }
  else if (readStr.startsWith(F("+KCNX_IND: 1,2")))                                         //PDP connection status 2:Failed to connect
  {
    return 1;
  }
  else if (readStr.startsWith(F("+KCNX_IND: 1,0")) || readStr.startsWith(F("+KCNX_IND: 1,3")))  //PDP connection status 0:Disconnected due to network 3:Closed 
  { 
    return 2;
  }
  else if (readStr.startsWith(F("+KCNX_IND: 1,5")) || readStr.startsWith(F("+KCNX_IND: 1,6")))
  {
	  goto ret4;
  }
  else if  (readStr.startsWith(F("+KTCP_NOTIF: 1,10")))                                  //
  {
     return 0;
  }
  else if  (readStr.startsWith(F("+KTCP_NOTIF: 1,")))
  {
     return 2;
  }  
  else
  {
    goto ret4;
  }
  return 3;
}
/**********************************************
*  AT＋CCLK commandの応答確認
**********************************************/
int chkCclkMes()
{
  String readStr;
  String readStrtmp;
  unsigned long sTime;
  unsigned long cTime;
  sTime = millis();
ret6:
  cTime = millis();
  if (cTime >= sTime){
    if ( cTime - sTime > 10000) return 2;
  }else {
      if ((cTime + (4294967295 -  sTime)) > 10000)  return 2;
  }
  if (SerialHL7800.available() <= 0) goto ret6;
  readStr="";
  readStrtmp = SerialHL7800.readStringUntil('\n');  
  readStr = readStr + readStrtmp;
  while (!(readStrtmp.endsWith(F("\n")) || readStrtmp.endsWith(F("\r"))))
  {
    readStrtmp = SerialHL7800.readStringUntil('\n'); 
    readStr = readStr + readStrtmp;    
  }
  readStr.replace(F("\r"), F(""));
  readStr.replace(F("\n"), F(""));
  readStr.trim();      
#if USB_EN  
  Serial.println(readStr);
#endif
  if (readStr.startsWith(F("OK")))
  {
      return 0;
  }
  else if (readStr.startsWith(F("+CCLK:"))) { 
    int val = readStr.indexOf(F("+"));
    getTime = readStr.substring(8,25);
#if USB_EN  
  Serial.print(F("Get time = ")); 
  Serial.println(getTime);
  
#endif
   goto ret6;
  }
  else if (readStr.startsWith(F("ERROR")) || readStr.startsWith(F("+CME ERROR")))
  {
      return 1;
  }
  goto ret6;  
  return 3;
}
/**********************************************
*  送信データ受信応答確認
**********************************************/
int chkRsvMes()
{
  String readStr;
  String readStrtmp;
  unsigned long sTime;
  unsigned long cTime;
  char mesdata[40];
  int datalength = 0;

  sTime = millis();
ret5:
  cTime = millis();
  if (cTime >= sTime){
    if ( cTime - sTime > 10000) return 2;
  }else {
      if ((cTime + (4294967295 -  sTime)) > 10000)  return 2;
  }
  if (SerialHL7800.available() <= 0) goto ret5;
  readStr="";
  readStrtmp = SerialHL7800.readStringUntil('\n');  
  readStr = readStr + readStrtmp;
  while (!(readStrtmp.endsWith(F("\n")) || readStrtmp.endsWith(F("\r"))))
  {
    readStrtmp = SerialHL7800.readStringUntil('\n'); 
    readStr = readStr + readStrtmp;    
  }
  readStr.replace(F("\r"), F(""));
  readStr.replace(F("\n"), F(""));
  readStr.trim();      
#if USB_EN  
  Serial.println(readStr);
#endif
  if (readStr.startsWith(F("+KTCP_DATA: 1,"))) {
    readStr.toCharArray(mesdata,40);
    strtok(mesdata, ",");
    datalength =  atoi(strtok(NULL, ","));
#if true
	// ホストサーバからデータ受信要求バイト数分受信する
    if ( datalength > 0 )
    {
      writeCommand("AT+KTCPRCV=1," +  String(datalength));
      chkRtnMes(F("OK"));
      return 0;
    }
#endif
    return 0;
  }
  goto ret5;
  return 3;
}

/**********************************************
*  センサーデータ取得＆データ送信
**********************************************/
int sendSenserData()
{
  int ret;
  float value;
  char temp[6], humid[6], light[6],battVolt[5];
  char sendDataBuf[200];
  char strBuf[25];
  int sendDatalen = 0;
  int retryCnt = 0;

  //get Senser data
  getSenserData();
  writeCommand(F("AT+CCLK?"));
  //chkRtnMes(F("OK"));
  chkCclkMes();

//-------------------------
  // Temperature (4Byte)
  //-------------------------
  value = dataTemp;

  if(value >= 100){
    value = 99.99;
  }
  else if(value <= -10){    
    value = -9.99;
  } 
  dtostrf(value,-5,2,temp);

  //-------------------------
  // Humidity (4Byte)
  //-------------------------
 if(value >= 100){
    value = 99.99;
  }
  else if(value <0){    
    value = 0;
  } 
  value = dataHumid;   
  dtostrf(value,-5,2,humid);
  
  //-------------------------
  // Ambient Light (5Byte)
  //-------------------------
  value = dataLight;
    
  if(value >= 100000){
    value = 99999;
  }
  else if (value <0){
	  value = 0;
  }
  dtostrf(value,-5,0,light);
  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatVolt;

  if (value >= 10){
   value = 9.99;
  }
  else if(value < 0){    
    value = 0;
  } 
  dtostrf(value, -4, 2, battVolt);  

  datatrim(temp);
  datatrim(humid);
  datatrim(light); 
  datatrim(battVolt);
 
  sprintf(sendDataBuf,"?value1=%s&value2=%s,%s,%s,%s",getTime.c_str(),temp,humid,light,battVolt);
  
  sendDatalen = strlen(sendDataBuf);
  sprintf(strBuf,"Content-Length: %d",sendDatalen);

#if USB_EN
  Serial.println("Send data = " + String(sendDataBuf));
  Serial.println("Send data lenght = " + String(strBuf));
#endif  
 ret3:
  //Start TCP Connection
  writeCommand(F("AT+KTCPCNX=1"));
  if( chkRtnMes(F("OK")))
  {
#if USB_EN    
    Serial.println(F("ERROR STOPED : +KTCPCNX ERROR"));    
#endif
    errMes(F("COM ERR"),F("KTCPCNX"));        
    writeCommand(F("AT+KTCPDEL=1"));
    chkRtnMes(F("OK"));
	delay(500);
    writeCommand_KCNXCFG(apn,user_name,password);    
    chkRtnMes(F("OK"));
	delay(500);
   writeCommand(F("AT+KTCPCFG=1,0,\"maker.ifttt.com\",80"));
    chkRtnMes(F("OK"));
    return 1;   
  }
  ret = chkKcnxMes();
  {
    if ( ret == 1){
      if (retryCnt < 2) {
        errMes(F("CNX ERR"),F("RETRY"));
        delay(200);  
        retryCnt ++;
        goto ret3;
      }
      else{
        errMes(F("CNX ERR"),F("RTRY OVR"));
        writeCommand(F("AT+KTCPDEL=1"));      
        chkRtnMes(F("OK"));
        delay(500); 
        writeCommand_KCNXCFG(apn,user_name,password);
        chkRtnMes(F("OK"));
		delay(500);
        writeCommand(F("AT+KTCPCFG=1,0,\"maker.ifttt.com\",80"));
        chkRtnMes(F("OK"));      
       return 1;
      }
    }else if ( ret > 1){
      errMes(F("CNX ERR"),F("OTHER"));
      writeCommand(F("AT+KTCPDEL=1"));      
      chkRtnMes(F("OK"));
      delay(500); 
      writeCommand_KCNXCFG(apn,user_name,password);
      chkRtnMes(F("OK"));
	  delay(500);
      writeCommand(F("AT+KTCPCFG=1,0,\"maker.ifttt.com\",80"));
      chkRtnMes(F("OK"));      
      return 1;
    }    
  }

  //send
   writeCommand(F("AT+KTCPSND=1,400")); 
  if( chkRtnMes(F("CONNECT")))
  {
#if USB_EN    
    Serial.println(F("ERROR STOPED : +KTCPSND ERROR"));    
#endif
    errMes(F("SND ERR"),F("NO CARR"));    
    writeCommand(F("AT+KTCPCLOSE=1,1"));
    chkRtnMes(F("OK"));
    return 1;     
  }
  while (digitalRead(CTS) == 1){ 
	  returnCommand();
	  delay(1);
  }
  delay(1000);
  returnCommand();
  digitalWrite(RTS, HIGH);
  delay(200); 
 
#if 0 
  SerialHL7800.print(F("POST ")); 
  SerialHL7800.print(url);
  SerialHL7800.println(F(" HTTP/1.1"));
  SerialHL7800.println(F("Host: maker.ifttt.com"));
  SerialHL7800.println(F("Content-Type: application/json"));
	SerialHL7800.println(strBuf);
  SerialHL7800.println(F("Connection: close\r\n"));
  SerialHL7800.println();
  SerialHL7800.println(sendDataBuf);
  SerialHL7800.println(F("--EOF--Pattern--"));  
#else
  SerialHL7800.print(F("GET ")); 
  SerialHL7800.print(F("/trigger/"));
  SerialHL7800.print(event_name);
  SerialHL7800.print(F("/with/key/"));
  SerialHL7800.print(key);
  SerialHL7800.print(sendDataBuf);
  SerialHL7800.println(F(" HTTP/1.1"));
  SerialHL7800.println(F("Host: maker.ifttt.com"));
  SerialHL7800.println(F("Connection: close\r\n"));
  SerialHL7800.println();
  SerialHL7800.println(F("--EOF--Pattern--"));
#endif    

  delay(200);
  digitalWrite(RTS, LOW);
    
 delay(1000); 
  writeCommand(F("AT+KTCPCLOSE=1,1"));
  returnCommand();  

  return 0;
}
/**********************************************
*  LCDリーフへのエラーメッセージ表示
**********************************************/
void errMes(String emes1 , String emes2)
{
#if LCD
   lcd.clear();
   lcd.print(emes1);
   lcd.setCursor(0, 1);
   lcd.print(emes2); 
#endif    
}
//---------------------------------------
// trim 
// 文字列配列からSPを削除する
//---------------------------------------
void datatrim(char * data)
{
  int i = 0, j = 0;

  while (*(data + i) != '\0'){
    if (*(data + i) != ' '){
      *(data + j) = *(data + i);
      j++;
    }
    i++;
  }
  *(data + j) = '\0';
}
/**********************************************
*  Reset HL7800
**********************************************/
void resetHL7800()
{

#if USB_EN    
  Serial.println(F("############### RESET HL7800 ###############"));       
#endif
#if LCD
 lcd.clear();
 lcd.print(F("HL7800"));
 lcd.setCursor(0, 1);
 lcd.print(F("RESTING!"));
#endif

 //HL7800 リセットシーケンス 
  digitalWrite(RST_N, LOW);
  delay(20);  //20ms  
  digitalWrite(RST_N, HIGH);
  delay(200); //200ms
  
#if false
  int i = 0;
  for (i = 15; i > 0; i--){
#if USB_EN    
	  Serial.println(i, DEC);
#endif      
	  delay(1000);
  }
#else
  while (digitalRead(DSR) == 0){
	  delay(100);
  }
  Serial.println(F("DSR is LOW"));
  while (digitalRead(CTS) == 1){
	  delay(100);
  }
  Serial.println(F("CTS is HIGH"));
  delay(5000);
  writeCommand(F("AT"));
  while (chkRtnMes(F("OK")) != 0){
	  delay(1000);
	  writeCommand(F("AT"));
  }
#endif 
  delay(1000);
  writeCommand(F("AT+CFUN=1"));
  chkRtnMes(F("OK"));
  delay(500);
#if USB_EN
  writeCommand(F("ATE1"));
#else
  writeCommand(F("ATE0"));
#endif
  chkRtnMes(F("OK"));

  delay(500);
#if LCD
 lcd.clear();
 lcd.print(F("RESET"));
 lcd.setCursor(0, 1);
 lcd.print(F("END"));
#endif  
  writeCommand_KCNXCFG(apn,user_name,password);
  chkRtnMes(F("OK"));
  delay(500);
  writeCommand(F("AT+KTCPCFG=1,0,\"maker.ifttt.com\",80"));
  chkRtnMes(F("OK"));
  delay(1000);
}
/**********************************************
*  Start up
**********************************************/
void setup()
{
#if USB_EN  
  Serial.begin(115200); 
  Serial.println(F("setup start"));
#endif  
  SerialHL7800.begin(9600);

  //AVR IO PORT 設定
  pinMode(WAKEUP, OUTPUT);
  digitalWrite(WAKEUP, LOW);
  pinMode(RI, INPUT);
  pinMode(PWR_ON_N, OUTPUT); 
  digitalWrite(PWR_ON_N, LOW); 
  pinMode(DTR, OUTPUT);
  digitalWrite(DTR, HIGH);
  pinMode(DSR, INPUT);
  pinMode(CTS, INPUT);
  pinMode(RTS, OUTPUT);
  digitalWrite(RTS, LOW);
  pinMode(INT0_N, INPUT);

  pinMode(RST_N, OUTPUT);
  digitalWrite(RST_N, LOW);

  //
#if LCD
 pinMode(INT0_N, INPUT);
 sw_sts_old = digitalRead(2);
#else
  attachInterrupt(digitalPinToInterrupt(A0),intSw , FALLING );
#endif 

  // IO　Expander 初期化
 Wire.begin();
 i2c_write_byte(I2C_EXPANDER_ADDR, 0x03, 0xf5); //Configuration register 1:inpit 0:output
 i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x00); // Output Port  register 

 //HL7800 Reset解除
  delay(20);
  digitalWrite(RST_N, HIGH);
  
#if LCD
 i2c_write_byte(I2C_EXPANDER_ADDR_LCD, 0x03, 0xFE);
 i2c_write_byte(I2C_EXPANDER_ADDR_LCD, 0x01, 0x01);
 
 lcd.begin(8, 2);
 lcd.setContrast(30);
 lcd.clear();
 lcd.print(F("NOW"));
 lcd.setCursor(0, 1);
 lcd.print(F("BOOTING!"));
 
#else
 i2c_write_byte(I2C_GROVE_ADDR, 0x03, 0xFE);
 i2c_write_byte(I2C_GROVE_ADDR, 0x01, 0x01);
#endif

  noInterrupts();
  setupTC2Int();
  interrupts();

  //Senser setup
  setupSensor();

#if USB_EN
  unsigned char ioVal;
  unsigned short battVal;

  char buf[40];
  Serial.println(F("setup end"));
  Serial.println(F(""));
  ioVal = i2c_read_byte(I2C_EXPANDER_ADDR, 0x00);
  sprintf(buf, "I/O expander port sts(reg0) = %02Xh", ioVal);
  Serial.println(buf); 
  
  i2c_read(I2C_ADC_ADDR, 0x00, 2, i2c_receiveBuf);
  battVal = ((i2c_receiveBuf[0] << 4 ) | ((i2c_receiveBuf[1] &0xF0) >> 4)) * 2 *(3300 / 256);
  sprintf(buf, "Battery Volt = %d mV", battVal );
  Serial.println(buf);
  Serial.println(F(""));
#endif    

  digitalWrite(DTR, LOW); 

#if USB_EN  
  Serial.println(F("Now booting just wait!")); 
#endif

#if false
  int i = 0;
  for (i = 30 ; i > 0 ;i--){
#if USB_EN    
      Serial.println(i,DEC);       
#endif      
      delay(1000);  
  }
#else
  while(digitalRead(DSR) == 0){
   delay(100); 
  }
  Serial.println(F("DSR is LOW"));   
  while (digitalRead(CTS) == 1){
	  delay(100);
  }
    Serial.println(F("CTS is HIGH"));
  delay(6000);
  writeCommand("AT");
  while(chkRtnMes(F("OK")) != 0){
      delay(1000); 
   writeCommand(F("AT"));
 }
#endif 
  delay(1000);  
  writeCommand(F("AT+CFUN=4"));
  chkRtnMes(F("OK"));
  delay(100);
  writeCommand(F("AT+KBNDCFG=0,00000000000000020000"));
  chkRtnMes(F("OK"));
  delay(100);
  writeCommand(F("AT+CEREG=2"));
  chkRtnMes(F("OK"));
  delay(100); 
  writeCommand_CGDCONT(apn);
  chkRtnMes(F("OK"));
  writeCommand(F("AT+KRIC=0"));
  chkRtnMes(F("OK"));
  delay(100);
  delay(1000);
  writeCommand(F("AT+KSLEEP=2"));
  chkRtnMes(F("OK")); 
   delay(1000);
  writeCommand(F("AT+CFUN=1"));
  chkRtnMes(F("OK"));
  delay(500);
 
#if USB_EN
  writeCommand(F("ATE1"));
#else
  writeCommand(F("ATE0"));
#endif
  chkRtnMes(F("OK"));
  delay(500);

#if USB_EN
  writeCommand(F("ATI3"));
  chkRtnMes(F("OK"));
  delay(500);
#endif

  /*******************/
  //接続するAPN設定
  writeCommand_KCNXCFG(apn,user_name,password);
  /*******************/
#if true
  if( chkRtnMes(F("OK")))
  {
#if USB_EN    
     Serial.println(F("ERROR STOPED : +KCNXCFG ERROR"));    
#endif
    errMes(F("ERR STOP"),F("KCNXCFG"));
    writeCommand(F("AT+CPOF")); 
    chkRtnMes(F("OK")); 
    while(1);   
  }
#endif

  delay(500);
  /*******************/
  //接続するサーバ設定　接続するサーバのIPアドレスを設定
  writeCommand(F("AT+KTCPCFG=1,0,\"maker.ifttt.com\",80"));
  /*******************/
  if( chkRtnMes(F("OK")))
  {
#if USB_EN    
    Serial.println(F("ERROR STOPED : +KTCPCFG ERROR"));    
#endif
    errMes(F("ERR STOP"),F("KTCPCFG"));
    writeCommand(F("AT+CPOF")); 
    chkRtnMes(F("OK"));
    while(1);   
  }

  writeCommand(F("AT+CCLK?"));
   chkRtnMes(F("OK"));
   MsTimer2::start();      // Timer2 inverval start

#if LCD
 lcd.clear();
 lcd.print(F("Ready!!"));
 lcd.setCursor(0, 1);
 lcd.print(F("PUSH SW1"));
#endif


}

/**********************************************
*  main loop
**********************************************/
int rsts;
int err_cnt = 0;
void loop()
{
  
  if (sendSts == 1) { 
    if (readSenser == 1){
#if LCD
      lcd.clear();
      lcd.print(F("Data"));
      lcd.setCursor(0, 1);
      lcd.print(F("sending")); 
#endif      
      rsts = sendSenserData();
      if (rsts) {
        if (err_cnt > 1)
        {
          err_cnt = 0;
          resetHL7800();
        }else{           
          delay(5000);
        }
        err_cnt++;
      }
      readSenser = 0;
	  lcd_view = 0;
	  lcd_chg = 0;
    }
#if LCD    
	if (lcd_chg == 1)
	{
		lcd_chg = 0;
		lcd.clear();
		if (lcd_view == 0){
			lcd.print(F("Temp"));
			lcd.setCursor(0, 1);
			lcd.print(String(dataTemp) + F(" C"));
		}
		else if (lcd_view == 1){
			lcd.print(F("Humidity"));
			lcd.setCursor(0, 1);
			lcd.print(String(dataHumid) + F(" %"));
		}
		else if (lcd_view == 2){
			lcd.print(F("Luminous"));
			lcd.setCursor(0, 1);
			lcd.print(String(dataLight) + F(" lx"));
		}
		else if (lcd_view == 3){
			lcd.print(F("Battery"));
			lcd.setCursor(0, 1);
			lcd.print(String(dataBatVolt) + F(" V"));
		}
		lcd_view++;
		if (lcd_view > 3) lcd_view = 0;
	}
#endif  
  }
#if true
  while(SerialHL7800.available() > 0) {       // 受信したデータが存在する
    str = SerialHL7800.readStringUntil('\n'); // 受信データを読み込む
#if USB_EN     
    Serial.println(str); 
#endif     
  }
#else
  timercounter = 0;
  for (timercounter = 0 ; timercounter < 60 ; timercounter ++)
  {
    while(SerialHL7800.available() > 0) { // 受信したデータが存在する
      str = SerialHL7800.readStringUntil('\n'); // 受信データを読み込む 
#if USB_EN      
      Serial.println(str);  
#endif      
      delay(1000);    
    }
  }
#endif
  int val = digitalRead(2);
  if (val == 1) {
     if (sw_sts_old == 0)
     {
      intSw();
     }
  }
  sw_sts_old = val;

}
/**********************************************
*  SW1 割り込み
**********************************************/
void intSw()
{
  if (sendSts == 0) {
    sendSts =1;
    readSenser = 1;
    timercounter = 0;
    lcd_chg = 0;
    lcd_view = 0;
#if USB_EN    
    Serial.println(F("data send start"));
#endif    
  }else{
    sendSts = 0;
#if USB_EN    
    Serial.println(F("data send stop"));
#endif    
#if LCD
    lcd.clear();
    lcd.print(F("SendStop"));
    lcd.setCursor(0, 1);
    lcd.print(F("STRT:SW1"));
#endif    
  }
}
