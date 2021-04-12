#if defined(_AVR_)
#include <util/delay.h>
#endif

#include "BME280.h"
#include <Wire.h>
#include <SPI.h>

BME280::BME280(){
	parameter.I2CAddress = 0x77;
	parameter.sensorMode = 0b11;
	parameter.IIRfilter;
	parameter.tempOversampling;
	parameter.pressOversampling;
	parameter.humidOversampling;
	parameter.tStandby = 0b000;
}


uint8_t BME280::init(void){

	//Initialize i2c
	Wire.begin();

	//Read Coefficients to calibrate output data to actual values
	readCoefficients();

	//BME280 has an internal filter with coefficients  of 2, 4, 8 or 16.
	writeConfig();

	//Set measurement rates
	setMeas();

	//Check the sensor is actually there.
	return checkAddress();
}

//### INIT FUNCTIONS ###
void BME280::readCoefficients(void){
	//Values are stored in 2 registers. Must left shift msb by 8 and then add lsb to get full value.
	bme280_coefficients.dig_T1 = ((uint16_t)(readByte(BME280_DIG_T1_MSB) << 8) + readByte(BME280_DIG_T1_LSB));
	bme280_coefficients.dig_T2 = ((int16_t)(readByte(BME280_DIG_T2_MSB) << 8) + readByte(BME280_DIG_T2_LSB));
	bme280_coefficients.dig_T3 = ((int16_t)(readByte(BME280_DIG_T3_MSB) << 8) + readByte(BME280_DIG_T3_LSB));

	bme280_coefficients.dig_P1 = ((uint16_t)(readByte(BME280_DIG_P1_MSB) << 8) + readByte(BME280_DIG_P1_LSB));
	bme280_coefficients.dig_P2 = ((int16_t)(readByte(BME280_DIG_P2_MSB) << 8) + readByte(BME280_DIG_P2_LSB));
	bme280_coefficients.dig_P3 = ((int16_t)(readByte(BME280_DIG_P3_MSB) << 8) + readByte(BME280_DIG_P3_LSB));
	bme280_coefficients.dig_P4 = ((int16_t)(readByte(BME280_DIG_P4_MSB) << 8) + readByte(BME280_DIG_P4_LSB));
	bme280_coefficients.dig_P5 = ((int16_t)(readByte(BME280_DIG_P5_MSB) << 8) + readByte(BME280_DIG_P5_LSB));
	bme280_coefficients.dig_P6 = ((int16_t)(readByte(BME280_DIG_P6_MSB) << 8) + readByte(BME280_DIG_P6_LSB));
	bme280_coefficients.dig_P7 = ((int16_t)(readByte(BME280_DIG_P7_MSB) << 8) + readByte(BME280_DIG_P7_LSB));
	bme280_coefficients.dig_P8 = ((int16_t)(readByte(BME280_DIG_P8_MSB) << 8) + readByte(BME280_DIG_P8_LSB));
	bme280_coefficients.dig_P9 = ((int16_t)(readByte(BME280_DIG_P9_MSB) << 8) + readByte(BME280_DIG_P9_LSB));

	bme280_coefficients.dig_H1 = ((uint8_t)(readByte(BME280_DIG_H1)));
	bme280_coefficients.dig_H2 = ((int16_t)(readByte(BME280_DIG_H2_MSB) << 8) + readByte(BME280_DIG_H2_LSB));
	bme280_coefficients.dig_H3 = ((uint8_t)(readByte(BME280_DIG_H3)));
	bme280_coefficients.dig_H4 = ((int16_t)((readByte(BME280_DIG_H4_MSB) << 4) + (readByte(BME280_DIG_H4_LSB) & 0x0F)));
	bme280_coefficients.dig_H5 = ((int16_t)((readByte(BME280_DIG_H5_MSB) << 4) + ((readByte(BME280_DIG_H5_LSB) >> 4) & 0x0F)));
	bme280_coefficients.dig_H6 = ((uint8_t)(readByte(BME_DIG_H6)));
}

void BME280::writeConfig(void){
	byte value;
	//filter register is shared with other bytes
	value = ((parameter.IIRfilter << 2) | (parameter.tStandby << 5))& 0b11111100;
	writeByte(BME280_config, value);
}

void BME280::setMeas(void){
	//Set Humidity Oversampling
	byte value;
	value = parameter.humidOversampling & 0b00000111;
	writeByte(BME280_ctrl_hum, value);

	//set Temp, Press Oversampling, sensor mode
	value = (parameter.tempOversampling << 5) & 0b11100000;
	value |= (parameter.pressOversampling << 2) & 0b00011100;
	value |= (parameter.sensorMode) & 0b00000011;
	writeByte(BME280_ctrl_meas, value);
}

uint8_t BME280::checkAddress(void){
	uint8_t value = readByte(BME280_chip_id);
	return value
}

bool BME280::isMeasuring(void){
	uint8_t value = readByte(BME280_status);
	return(value & (1<<3))
}

//### Sensor Read Functions ###

float BME280::readTempC(void){
	//Pull data from registers
	int32_t adc_T, var1, var2, T;
	uint8_t regData[3]
	readMultiBytes(BME280_temp_msb, 3, regData);
	adc_T = (uint32_t)regData[0] << 12;
	adc_T |= (uint32_t)regData[1] << 4;
	adc_T |= ((uint32_t)regData[2] >> 4 & 0b00001111);

	//Calculate actual temperature from equations in datasheet. Returns Temperature in Celsius
	var1 = ((((adc_T>>3) - ((int32_t)bme280_coefficients.dig_T1<<1))) * ((int32_t)bme280_coefficients.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bme280_coefficients.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280_coefficients.dig_T1))) >> 12) * 
		((int32_t)bme280_coefficients.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	T = T/100;
	return (float)T;
}

float BME280::readPress(void){
	int32_t adc_P;
	uint8_t regData[3];
	readMultiBytes(BME280_press_msb, 3, regData);
	adc_P = (uint32_t)regData[0] << 12;
	adc_P |= (uint32_t)regData[1] << 4;
	adc_P |= ((uint32_t)regData[2] >> 4 & 0b00001111);

	//Returns pressure in Pa
	int64_t var1, var2, P;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_coefficients.dig_P6;
	var2 = var2 + ((var1*(int64_t)bme280_coefficients.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280_coefficients.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280_coefficients.dig_P3)>>8) + ((var1 * (int64_t)bme280_coefficients.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)bme280_coefficients.dig_P1)>>33;

	if (var1==0){
		return 0 //Exception handling when dviding by zero
	}

	P = 1048576 - adc_P;
	P = (((P<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bme280_coefficients.dig_P9) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)bme280_coefficients.dig_P8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)bme280_coefficients.dig_P7)<<4);
	P = P/256.0;
	return (float)P;
}

float BME280::readHumid(void){
	int32_t adc_H;
	uint8_t regData[2];
	readMultiBytes(BME280_hum_msb, 2, regData);
	adc_H = (int32_t)regData[0] << 8;
	adc_H |= (int32_t)regData[1];

	//Returns humidity in RH
	int32_t var1, H;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)bme280_coefficients.dig_H4) << 20) - (((int32_t)bme280_coefficients.dig_H5) * var1)) + 
		((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)bme280_coefficients.dig_H6)) >> 10) * (((var1 * 
		((int32_t)bme280_coefficients.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
		((int32_t)bme280_coefficients.dig_H2) + 8129) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)bme280_coefficients.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
	H = ((float)var1>>12);
	H = H/1024.0;
	return (float)H;
}

void BME280::readAll(BME280_Measurements *measurements){
	uint8_t regData[8]
	readMultiBytes(BME280_press_msb, 8, regData);

	int32_t adc_T, adc_P, adc_H;
	int64_t var1, var2, P, T, H;

	//Temperature
	adc_T = (uint32_t)regData[3] << 12;
	adc_T |= (uint32_t)regData[4] << 4;
	adc_T |= ((uint32_t)regData[5] >> 4 & 0b00001111);

	//Calculate actual temperature from equations in datasheet. Returns Temperature in Celsius
	var1 = ((((adc_T>>3) - ((int32_t)bme280_coefficients.dig_T1<<1))) * ((int32_t)bme280_coefficients.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bme280_coefficients.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280_coefficients.dig_T1))) >> 12) * 
		((int32_t)bme280_coefficients.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	T = T/100;

	//Pressure
	adc_P = (uint32_t)regData[0] << 12;
	adc_P |= (uint32_t)regData[1] << 4;
	adc_P |= ((uint32_t)regData[2] >> 4 & 0b00001111);

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_coefficients.dig_P6;
	var2 = var2 + ((var1*(int64_t)bme280_coefficients.dig_P5)<<17);
	var2 = var2 + (((int64_t)bme280_coefficients.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bme280_coefficients.dig_P3)>>8) + ((var1 * (int64_t)bme280_coefficients.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)bme280_coefficients.dig_P1)>>33;

	if (var1==0){
		return 0 //Exception handling when dviding by zero
	}

	P = 1048576 - adc_P;
	P = (((P<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bme280_coefficients.dig_P9) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)bme280_coefficients.dig_P8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)bme280_coefficients.dig_P7)<<4);
	P = P/256.0;

	adc_H = regData[6] << 8;
	adc_H |= regData[7];
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)bme280_coefficients.dig_H4) << 20) - (((int32_t)bme280_coefficients.dig_H5) * var1)) + 
		((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)bme280_coefficients.dig_H6)) >> 10) * (((var1 * 
		((int32_t)bme280_coefficients.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
		((int32_t)bme280_coefficients.dig_H2) + 8129) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)bme280_coefficients.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);
	H = ((float)var1>>12);
	H = H/1024.0;

	measurements->temp = (float)T;
	measurements->press = (float)P;
	measurements->humid = (float)H;
}


//### HELPER FUNCTIONS ###
uint8_t BME280::readByte(byte reg){
	//Uses i2c to communicate. See https://www.arduino.cc/en/reference/wire for docs.
	uint8_t value;

	Wire.beginTransmission(parameter.I2CAddress);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(parameter.I2CAddress,1);
	value = Wire.read();
	return value;
}

void BME280::readMultiBytes(byte reg, uint8_t length, uint8_t *outputPointer){
	uint8_t iter = 0;
	uint8_t value;

	Wire.beginTransmission(parameter.I2CAddress);
	Wire.write(reg);
	Wire.endTransmission();
	while ((Wire.available()) && (iter<length)){
		value = Wire.read();
		*outputPointer = value;
		outputPointer++;
		i++;
	}
}

void BME280::writeByte(byte reg, byte data){
	Wire.beginTransmission(parameter.I2CAddress);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}