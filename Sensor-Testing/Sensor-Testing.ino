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
