// working compass demo
/*
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  compass.setCalibrationOffsets(-270.00, 1475.00, 2373.00);
  compass.setCalibrationScales(1.22, 1.20, 0.74);
  
}

void loop() {
  int a;
  
  // Read compass values
  compass.read();

  // Return Azimuth reading
  a = compass.getAzimuth();
  
  Serial.print("A: ");
  Serial.print(a);
  Serial.println();
  
  delay(250);
}
*/

/*
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// A test script to get data readings from the various sensors
// used in this project.

#define GPS_TX 10
#define GPS_RX 9
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop()
{
  while (gpsSerial.available() > 0)
    {
      char c = gpsSerial.read();
      Serial.println(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        Serial.println("VALID");
    }
} */



/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */

/*
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_TX 17
#define GPS_RX 16

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial gpsSerial(1);

void setup(){
  Serial.begin(9600);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0){
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    }
  }
}
*/

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
 Serial.begin(115200);
 //SerialBT.begin("ESP32test"); //Bluetooth device name
 Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
 /*if (Serial.available()) {
   SerialBT.write(Serial.read());
 }
 if (SerialBT.available()) {
   Serial.write(SerialBT.read());
 }
 delay(20);*/
}