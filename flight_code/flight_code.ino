#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunLSM9DS1.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

const int cs_gps = 15;
const int cs_sd = 14;
const int cs_radio = 22;
const int radio_interrupt = 6;
const int radio_address_rocket = 1;
const int radio_address_ground = 2;
const int radio_address_all = 255;
char data_string[80]; // maximum length for RF95 is 251 bytes
bool tx_success = false;
const int grn_led = 2;
const int red_led = 3;
const int bzzr = 23;
unsigned int last_read_sensors = 0;
unsigned int last_tx = 0;

BME280 bme;
LSM9DS1 imu;
BME280_SensorMeasurements measurements;
TinyGPSPlus gps;
RH_RF95 driver(cs_radio, radio_interrupt); //chip select and interrupt pin
RHReliableDatagram manager(driver, radio_address_rocket);

void errorTone();
void sirenTone();
void writeHeaders();
void readGPS();
bool readBME();
bool readIMU();
void writeBME(bool);
void writeIMU(bool);
void writeMicros();
void write2SD();
bool write2Radio();

void setup() {
  Serial.begin(9600);
  Serial.println("initializing");
  for (int i = 0; i < 2; i++){
    tone(bzzr, 1000);
    delay(500);
    noTone(bzzr);
    delay(250);
  }
  // Setup GPIO pins
  pinMode(grn_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(bzzr, OUTPUT);
  digitalWrite(grn_led, LOW);
  digitalWrite(red_led, LOW);
  
  // Setup SPI Bus
  SPI.begin();
  pinMode(cs_gps, OUTPUT);
  pinMode(cs_sd, OUTPUT);
  if (!SD.begin(cs_sd)){
    digitalWrite(grn_led, HIGH);
    errorTone();
    Serial.println("sd failure");
  }

  if (!manager.init()){
    digitalWrite(grn_led, HIGH);
    errorTone();
    Serial.println("radio failure");
  }
  if (driver.setFrequency(915.0)){
    digitalWrite(grn_led, HIGH);
    errorTone();
    Serial.println("frequency failure");
  }
  manager.setRetries(0);

  // Setup I2C Bus
  Wire.begin();
  Wire.setClock(400000);
  if (!bme.beginI2C()){
    digitalWrite(red_led, HIGH);
    errorTone();
  }
  bme.setStandbyTime(0);
  bme.setTempOverSample(16);
  bme.setPressureOverSample(16);
  
  if (!imu.begin()){
    digitalWrite(red_led, HIGH);
    errorTone();
  }
  imu.settings.gyro.scale = 2000;
  imu.settings.gyro.sampleRate = 6;
  imu.settings.accel.scale = 16;
  imu.settings.accel.sampleRate = 6;
  imu.settings.mag.scale = 12;
  imu.settings.mag.sampleRate = 5;

  // Calibrate IMU
  imu.calibrate(true);
  imu.calibrateMag(true);

  // Write header to SD file
  writeHeaders();
  //sirenTone();
  noTone(bzzr);
}

void loop() {
  bool newBME;
  bool newIMU;
  strcpy(data_string, "");
  
  // Poll gps at 10 Hz
  if (gps.location.age() > 100){
    readGPS();
  } else{
    strcpy(data_string, ",,,");
  }

  // Poll Pressure, Temperature, IMU at 100 Hz
  if (micros() - last_read_sensors > 10000){
    last_read_sensors = micros();
    newBME = readBME();
    newIMU = readIMU();
    writeBME(newBME);
    writeIMU(newIMU);
  }

  writeMicros();
  
  write2SD();
  Serial.println(data_string);

  if (micros() - last_tx > 1000){
    interrupts();
    //driver.send((uint8_t*)data_string, sizeof(data_string));
    //tx_success = write2Radio(radio_address_ground);
    last_tx = micros();
    noInterrupts();
  }
}

void errorTone(){
  tone(bzzr, 500, 2000);
  noTone(bzzr);
}

void sirenTone(){
  for (int k=0; k<3; k++){
    for (int j=1; j<=4; j++){
      tone(bzzr, 500*j);
      delay(500);
    }
    for (int j=4; j>0; j--){
      tone(bzzr, 500*j);
      delay(500);
    }
  }
}

void writeHeaders(){
  // Note: This does not create a new file each time the computer is powered on.
  // To denote between power cycles, a header line is written.
  String header = "Time,Lattitude,Longitude,Pressure,Temperature,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,ucTime";
  File dataFile = SD.open("test_log.txt", FILE_WRITE);
  if (dataFile){
    dataFile.println(header);
    dataFile.close();
  }
}

void readGPS(){
  char result = 0;
  char temp_time[10];
  char temp_long[10];
  char temp_lat[10];
  SPI.beginTransaction(SPISettings(550000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs_gps, LOW);

  do {
    result = SPI.transfer(0xFF);
    gps.encode(result);
  } while (result != 0xFF);

  if (gps.time.isValid()){
    sprintf(temp_time, "%.2d:%.2d:%.2d,", gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    strcpy(temp_time, "XX:XX:XX,");
  }
  strcpy(data_string, temp_time);

  if (gps.location.isValid()){
    dtostrf(gps.location.lat(), 8, 5, temp_lat);
    dtostrf(gps.location.lng(), 8, 5, temp_long);
  } else {
    strcpy(temp_lat, "X");
    strcpy(temp_long, "X");
  }

  strcat(data_string, temp_lat);
  strcat(data_string, ",");
  strcat(data_string, temp_long);
  strcat(data_string, ",");
}

bool readBME(){
  if (!bme.isMeasuring()){
    bme.readAllMeasurements(&measurements);
    return true;
  } else{
    return false;
  }
}

bool readIMU(){
  last_read_sensors = millis();
  bool success = false;
  if (imu.gyroAvailable()){
    imu.readGyro();
    success = true;
  }

  if (imu.accelAvailable()){
    imu.readAccel();
    success = true;
  }
  return success;
}

void writeBME(bool newData){
  char temp[10];
  if (newData){
    dtostrf(measurements.pressure, 6, 0, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");

    dtostrf(measurements.temperature, 4, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
  } else{
    strcat(data_string, ",,");
  }
}

void writeIMU(bool newData){
  char temp[10];
  if (newData){
    dtostrf (imu.calcGyro(imu.gx), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
    dtostrf(imu.calcGyro(imu.gy), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
    dtostrf(imu.calcGyro(imu.gz), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");

    dtostrf(imu.calcAccel(imu.ax), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
    dtostrf(imu.calcAccel(imu.ay), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
    dtostrf(imu.calcAccel(imu.az), 5, 2, temp);
    strcat(data_string, temp);
    strcat(data_string, ",");
  } else{
    strcat(data_string, ",,,,,,");
  }
}

void writeMicros(){
  char temp[10];
  sprintf(temp, "%lu", micros());
  strcat(data_string, temp);
}

void write2SD(){
  File dataFile = SD.open("test_log.txt", FILE_WRITE);

  if (dataFile){
    dataFile.println(data_string);
    dataFile.close();
  }
}

bool write2Radio(int address){
  //return driver.send((uint8_t*)data_string, sizeof(data_string));
  //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  // must cast the data_string as uint8_t
  driver.send((uint8_t*)data_string, sizeof(data_string));//, address);
  return false;
    //uint8_t len = sizeof(buf);
    //uint8_t from;
    //if (manager.recvfromAckTimeout(buf, &len, 2000, &from)){
      //return true;
    //}

  //return false;
}
