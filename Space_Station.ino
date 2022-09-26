#include <esp_now.h>
#include <WiFi.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include "DHTesp.h"
#include "Tarsier_RPI_1031.h"
const int DHT_PIN = 26;
DHTesp dhtSensor;
Adafruit_BMP280 bme;
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, gSqrt, mDirection, mX, mY, mZ;
int TILT_S1_PIN    = 4;
int TILT_S2_PIN    = 5;
Tarsier_RPI_1031 _dir_Sensor(TILT_S1_PIN, TILT_S2_PIN);
uint8_t broadcastAddress[] = {0x4C, 0xEB, 0xD6, 0x62, 0x05, 0xD4};
typedef struct struct_message {
  float temp1, hum1, atmPre1, avx1, avy1, avz1, av1, ax1, ay1, az1, acc1, mx1, my1, mz1, mag1, realAlt1;
} struct_message;
struct_message myData;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(9600);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  while (!Serial);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;    
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(21, 22);  // Wire.begin(sda, scl)
    mySensor.setWire(&Wire);
  #else
    Wire.begin();
    mySensor.setWire(&Wire);
  #endif
    bme.begin(0x76);
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
}

void loop() {
  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.print("Acceleration-X: " + String(aX));
    Serial.print("\tAcceleration-Y: " + String(aY));
    Serial.print("\tAcceleration-Z: " + String(aZ));
    Serial.println("\tAcceleration: " + String(aSqrt));
  }
  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    gSqrt = sqrt(gX*gX + gY*gY + gZ*gZ);
    Serial.print("Angular Velocity-X: " + String(gX));
    Serial.print("\tAngular Velocity-Y: " + String(gY));
    Serial.print("\tAngular Velocity-Z: " + String(gZ));
    Serial.println("\tAngular Velocity: " + String(gSqrt));
  }
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.print("Magnetic Field-X: " + String(mX));
    Serial.print("\tMagnetic Field-Y: " + String(mY));
    Serial.print("\tMagnetic Field-Z: " + String(mZ));
    Serial.println("\tMagnetic Field: " + String(mDirection));
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure());
    Serial.println(" Pa");
  }
  Serial.print("Altitude = ");
  Serial.print(bme.readAltitude(100000));
  Serial.println(" meters");
  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  Serial.println("Temperature: " + String(data.temperature, 2) + "°C");
  Serial.println("Humidity: " + String(data.humidity, 1) + "%");
  Serial.println("Done");
  delay(1000);
  myData.temp1 = data.temperature;
  myData.hum1 = data.humidity;
  myData.atmPre1 = bme.readPressure();
  myData.avx1 = gX;
  myData.avy1 = gY;
  myData.avz1 = gZ;
  myData.av1 = gSqrt;
  myData.ax1 = aX;
  myData.ay1 = aY;
  myData.az1 = aZ;
  myData.acc1 = aSqrt;
  myData.mx1 = mX;
  myData.my1 = mY;
  myData.mz1 = mZ;
  myData.mag1 = mDirection;
  myData.realAlt1 = bme.readAltitude(100000);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
}
