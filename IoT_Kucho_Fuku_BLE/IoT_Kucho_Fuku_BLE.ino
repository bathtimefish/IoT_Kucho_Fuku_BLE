/*
    note: need add library Adafruit_BMP280 from library manage
    Github: https://github.com/adafruit/Adafruit_BMP280_Library
*/

#include <M5StickC.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"

DHT12 dht12; 
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;
Adafruit_BMP280 bme;

/* Global Valiables */
int RELAYPIN = 32;
bool fanOn = false;
bool onRemote = false;
float hum = 0.0; // 湿度
float tmp = 0.0; // 温度
float prs = 0.0; // 気圧
int   fan = 0;   // FAN on/off
char msg[10];


void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();  
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();
  
  while((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();
    
    /* Update x-Axis max/min value */
    if(value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    } 
    else if(value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if(value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    } 
    else if(value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if(value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    } 
    else if(value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }
    
    Serial.print(".");
    delay(1);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min)/2;
  value_offset.y = value_y_min + (value_y_max - value_y_min)/2;
  value_offset.z = value_z_min + (value_z_max - value_z_min)/2;
}

void fanctrl(bool on)
{
  if(on) {
    if(fanOn == false) {
      digitalWrite(RELAYPIN, HIGH);
      fanOn = true;
      fan = 1;
      Serial.printf("%2.0f%% ", hum);
      Serial.println("FAN ON!");
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.print("FAN ON!");
    }
  } else {
    if(fanOn == true) {
      digitalWrite(RELAYPIN, LOW); 
      fanOn = false;
      fan = 0;
      Serial.printf("%2.0f%% ", hum);
      Serial.println("FAN OFF");
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.print("FAN OFF");
    }
  }
}

/* BLE Service */
#define SERVICE_UUID         "3ca95d2e-cb97-4f56-878e-aea7016b6fde"
#define CHARACTERISTIC_UUID1 "86fc1686-4de5-47d2-b3ad-c4bd38b64618"
#define CHARACTERISTIC_UUID2 "646ab2d1-d41f-429a-a3b6-d936e5b0059f"
#define CHARACTERISTIC_UUID3 "14f4fc65-4045-4c55-a766-c44b3ec00de1"
#define CHARACTERISTIC_UUID4 "32e06a5d-63b2-4913-95c2-31cd9a94af7c"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic1 = NULL;
BLECharacteristic* pCharacteristic2 = NULL;
BLECharacteristic* pCharacteristic3 = NULL;
BLECharacteristic* pCharacteristic4 = NULL;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      M5.Lcd.setCursor(0, 60, 2);
      M5.Lcd.println("Connected   ");
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      M5.Lcd.setCursor(0, 60, 2);
      M5.Lcd.println("Disconnected");
      deviceConnected = false;
    }
};

class TmpCb: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    sprintf(msg, "%2.1f", tmp);
    pCharacteristic->setValue(msg);
  }
};

class HumCb: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    sprintf(msg, "%2.0f", hum);
    pCharacteristic->setValue(msg);
  }
};

class PrsCb: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    sprintf(msg, "%2.1f", prs);
    pCharacteristic->setValue(msg);
  }
};

class FanCb: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    sprintf(msg, "%d", fan);
    pCharacteristic->setValue(msg);
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    //M5.Lcd.println(value.c_str());
    int on = strtol(value.c_str(), NULL, 10);
    if(on == 1) {
      fanctrl(true);
      onRemote = true;
    } else {
      fanctrl(false);
      onRemote = false;
    }
  }
};


void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0,26);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Axp.ScreenBreath(7); // Screen brightness level
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.println("IoT Kucho-Fuku");
  pinMode(M5_BUTTON_HOME, INPUT);

  /* Initialize RELAY Pin */
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, LOW);
  M5.Lcd.setCursor(0, 40, 2);
  M5.Lcd.print("FAN OFF");

  /* Initialize BLE */
  Serial.println("Starting BLE Server...");
  BLEDevice::init("IoTKuchoFuku");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic1 = pService->createCharacteristic( // tmp
                                         CHARACTERISTIC_UUID1,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic2 = pService->createCharacteristic( // hum
                                         CHARACTERISTIC_UUID2,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic3 = pService->createCharacteristic( // prs
                                         CHARACTERISTIC_UUID3,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic4 = pService->createCharacteristic( // fan
                                         CHARACTERISTIC_UUID4,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );
  pCharacteristic1->setCallbacks(new TmpCb());
  pCharacteristic1->addDescriptor(new BLE2902());
  pCharacteristic2->setCallbacks(new HumCb());
  pCharacteristic2->addDescriptor(new BLE2902());
  pCharacteristic3->setCallbacks(new PrsCb());
  pCharacteristic3->addDescriptor(new BLE2902());
  pCharacteristic4->setCallbacks(new FanCb());
  pCharacteristic4->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("Start BLE Server");
  M5.Lcd.setCursor(0, 60, 2);
  M5.Lcd.println("Disconnected");
  deviceConnected = false;

  /* Initialize BMM150 */
  if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }
  if (!bme.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }
  calibrate(10);
  Serial.print("\n\rCalibrate done..");
}

uint8_t setup_flag = 1;


void loop() {
  // put your main code here, to run repeatedly:
  tmp = dht12.readTemperature();
  hum = dht12.readHumidity();
  M5.Lcd.setCursor(0, 20, 2);
  M5.Lcd.printf("Temp: %2.1f Humi: %2.0f%%", tmp, hum);

  sprintf(msg, "%2.1f", tmp);
  pCharacteristic1->setValue(msg);
  pCharacteristic1->notify();
  sprintf(msg, "%2.0f", hum);
  pCharacteristic2->setValue(msg);
  pCharacteristic2->notify();
  sprintf(msg, "%2.1f", prs);
  pCharacteristic3->setValue(msg);
  pCharacteristic3->notify();
  sprintf(msg, "%d", fan);
  pCharacteristic4->setValue(msg);
  pCharacteristic4->notify();

  /* リモートでONされている場合は環境計測評価が無効となる */
  if(onRemote == false){
    /* 湿度55%を超えたらリレーをONにしてファンを起動する */
    if(hum > 55) fanctrl(true);
    /* 湿度50%未満ならリレーをOFFにしてファンを停止する */
    if(hum < 50) fanctrl(false);
  }

  /* Button pressed */
  if(M5.BtnB.wasPressed()){
    if(fanOn == false) {
      digitalWrite(RELAYPIN, HIGH);
      fanOn = true;
      Serial.println("FAN ON by BTN!");
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.print("FAN ON!");
      delay(3000);
      digitalWrite(RELAYPIN, LOW);
      fanOn = false;
      Serial.println("FAN OFF by BTN!");
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.print("FAN OFF!");
    }
  }
  M5.update();

  bmm150_mag_data value;
  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;

  float xyHeading = atan2(value.x, value.y);
  float zxHeading = atan2(value.z, value.x);
  float heading = xyHeading;

  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  float xyHeadingDegrees = xyHeading * 180 / M_PI;
  float zxHeadingDegrees = zxHeading * 180 / M_PI;
  
  prs = bme.readPressure();
  //M5.Lcd.setCursor(0, 60, 2);
  //M5.Lcd.printf("pressure: %2.1f", prs);

  delay(300);

  if(!setup_flag){
     setup_flag = 1;

     if(bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }
  if (!bme.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }
  calibrate(10);
  Serial.print("\n\rCalibrate done..");
 }


 if(digitalRead(M5_BUTTON_HOME) == LOW){
  setup_flag = 0;
  while(digitalRead(M5_BUTTON_HOME) == LOW);
 }

  
}
