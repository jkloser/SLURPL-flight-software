#ifndef BME280_H
#define BME280_H

#if (ARDUINO >= 100)
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Wire.h"
#include "SPI.h"

//Defines the memory register addresses of the sensor
#define BME280_chip_id    0xD0
#define BME280_hum_lsb    0xFE
#define BME280_hum_msb    0xFD
#define BME280_temp_xlsb  0xFC
#define BME280_temp_lsb   0xFB
#define BME280_temp_msb   0xFA
#define BME280_press_xlsb 0xF9
#define BME280_press_lsb  0xF8
#define BME280_press_msb  0xF7
#define BME280_config     0xF5
#define BME280_ctrl_meas  0xF4
#define BME280_status     0xF3
#define BME280_ctrl_hum   0xF2




struct DeviceParameter{
	uint8_t I2CAddress;
	uint8_t sensorMode;
	uint8_t IIRfilter;
	uint8_t tempOversampling;
	uint8_t pressOversampling;
	uint8_t humidOversampling;
	uint8_t tStandby;
};

struct BME280_Coefficients{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
};

struct BME280_Measurements{
	float temp
	float press
	float humid
};

enum Coefficients{
	//enum automatically assigns variables a value one greater than the previous
	//variable's value. These are the register addresses for coefficients for
	//calibrating the output values. LSB = least significatn bit, MSB = most
	BME280_DIG_T1_LSB = 0x88,
	BME280_DIG_T1_MSB,
	BME280_DIG_T2_LSB,
	BME280_DIG_T2_MSB,
	BME280_DIG_T3_LSB,
	BME280_DIG_T3_MSB,

	BME280_DIG_P1_LSB,
	BME280_DIG_P1_MSB,
	BME280_DIG_P2_LSB,
	BME280_DIG_P2_MSB,	
	BME280_DIG_P3_LSB,
	BME280_DIG_P3_MSB,
	BME280_DIG_P4_LSB,
	BME280_DIG_P4_MSB,
	BME280_DIG_P5_LSB,
	BME280_DIG_P5_MSB,
	BME280_DIG_P6_LSB,
	BME280_DIG_P6_MSB,
	BME280_DIG_P7_LSB,
	BME280_DIG_P7_MSB,
	BME280_DIG_P8_LSB,
	BME280_DIG_P8_MSB,
	BME280_DIG_P9_LSB,
	BME280_DIG_P9_MSB,

	BME280_DIG_H1     = 0xA1,
	BME280_DIG_H2_LSB = 0XE1,
	BME280_DIG_H2_MSB,
	BME280_DIG_H3,
	BME280_DIG_H4_LSB = 0xE5,
	BME280_DIG_H4_MSB = 0xE4,
	BME280_DIG_H5_LSB = 0xE5,
	BME280_DIG_H5_MSB = 0xE6,
	BME280_DIG_H6
};

class BME280{
public:
	DeviceParameter parameter;
	BME280_Coefficients bme280_coefficients;
	int32_t t_fine;

	BME280();
	uint8_t init(void);
	uint8_t readByte(byte reg);
	void writeByte(byte reg, byte data);
	void readMultiBytes(byte reg, uint8_t length, uint8_t *outputPointer);

	void readCoefficients(void);
	void writeConfig(void);
	void setMeas(void);
	uint8_t checkAddress(void);

	bool isMeasuring(void);
	float readTempC(void);
	float readPress(void);
	float readHumid(void);
	void readAll(BME280_Measurements *measurements);
}
#endif