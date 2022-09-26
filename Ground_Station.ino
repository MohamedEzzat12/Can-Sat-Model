#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  float temp1, hum1, atmPre1, avx1, avy1, avz1, av1, ax1, ay1, az1, acc1, mx1, my1, mz1, mag1, realAlt1;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println("Data received Successfully (•‿•)");
  Serial.print("Acceleration-X: " + String(myData.ax1));
  Serial.print("\tAcceleration-Y: " + String(myData.ay1));
  Serial.print("\tAcceleration-Z: " + String(myData.az1));
  Serial.println("\tAcceleration: " + String(myData.acc1));
  Serial.print("Angular Velocity-X: " + String(myData.avx1));
  Serial.print("\tAngular Velocity-Y: " + String(myData.avy1));
  Serial.print("\tAngular Velocity-Z: " + String(myData.avz1));
  Serial.println("\tAngular Velocity: " + String(myData.av1));
  Serial.print("Magnetic Field-X: " + String(myData.mx1));
  Serial.print("\tMagnetic Field-Y: " + String(myData.my1));
  Serial.print("\tMagnetic Field-Z: " + String(myData.mz1));
  Serial.println("\tMagnetic Field: " + String(myData.mag1));
  Serial.print("Pressure = ");
  Serial.print(myData.atmPre1);
  Serial.println(" Pa");
  Serial.print("Altitude = ");
  Serial.print(myData.realAlt1);
  Serial.println(" meters");
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(9600);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}
