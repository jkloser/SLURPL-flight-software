#include <TinyGPS++.h>
#include <SPI.h>

TinyGPSPlus gps;
const int cs = 15;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  pinMode(cs, OUTPUT);

//  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPSPlus::library_version());
  Serial.println();
}

void loop() {
  char result = 0;
  bool newData = false;
  
  SPI.beginTransaction(SPISettings(5500000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);
  do {
    result = SPI.transfer(0xFF);
    if (gps.encode(result)){
      newData = true;
    }
  } while (result != 0xFF);
  SPI.endTransaction();

  if (newData){
    Serial.println("New Data!");
    Serial.print(F("location: "));
    if (gps.location.isValid()){
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    } else{
      Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()){
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.year());
    } else{
      Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()){
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(F("."));
      if (gps.time.centisecond() < 10) Serial.print(F("0"));
      Serial.print(gps.time.centisecond());
    } else{
      Serial.print(F("INVALID"));
    }
    Serial.println();
  }
}
