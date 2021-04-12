#include <Wire.h>
#include "BME280.h"
BME280 bme = BME280();
BME280_Measurements measurements;

void setup(){
	Serial.begin(9600);
	Serial.println(F("BME 280 Example Code"));

	//########### SENSOR INITILIZATION ###########
	//Library is built for pressure (Pa), temperature (C), humidity (%) measurements over I2C.

	//Uncomment the following line if SDO is connected to GND.
	//bme.parameter.I2CAddress = 0x76;

	//Sensor Mode Options:
	//------------------------------
	//0b00		|		Sleep Mode
	//0b01		|		Forced Mode
	//0b11		|		Noraml Mode (default)
	//------------------------------
	//bme.parameter.sensorMode = 0b11;

	//IIR Filter Options:
	//------------------------------
	//0b000		|		Off
	//0b001		|		2
	//0b010		|		4
	//0b011		|		8
	//0b100		|		16 (default)
	//------------------------------
	//bme.parameter.sensorMode = 0b11;

	//Oversampling Options:
	//------------------------------
	//0b000		|		Off
	//0b001		|		1
	//0b010		|		2
	//0b011		|		4
	//0b100 	|		8
	//0b101 	|		16
	bme.parameter.tempOversampling = 0b101;
	bme.parameter.pressOversampling = 0b101;
	bme.parameter.humidOversampling = 0b011;

	//Standby time between measurements
	//Standby Options:
	//------------------------------
	//0b000		|		0.5 (ms) (default)
	//0b001		|		62.5
	//0b010		|		125
	//0b011		|		250
	//0b100 	|		500
	//0b101 	|		1000
	//0b110 	|		10
	//0b111 	|		20
	bme.parameter.tStandby = 0b000;

	if (bme280.init() != 0x60){
		Serial.println("BME280 could not be found.");
		while(1);
	}

	else{
		Serial.println("BME280 successfully initialized.");
		Serial.println();
	}
}

void loop(){
	bme.readAll(&measurements);
}