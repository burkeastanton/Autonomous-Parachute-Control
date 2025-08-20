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

/*
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// A test script to get data readings from the various sensors
// used in this project.

#define GPS_TX 17
#define GPS_RX 16
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
}
*/
