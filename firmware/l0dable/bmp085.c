/*!
 * \file bmp085.c
 * \brief Reads out the BMP085 pressure sensor.
 *
 * \author Andreas Müller <am@0x7.ch>
 * \date 1.8.2013
 * \version 0.1
 *
 * This code may be freely used and distributed under GNU GPL conditions.
 */

#include <sysinit.h>
#include <string.h>
#include <stdlib.h>

#include "basic/basic.h"
#include "basic/config.h"
#include "core/i2c/i2c.h"

#include "lcd/render.h"
#include "lcd/print.h"

#include "usetable.h"


#define BMP085_I2C_WRITE    0xEE
#define BMP085_I2C_READ     0xEF

#define BMP085_CONTROL_REG_ADDR	       0xf4
#define BMP085_CMD_MEASURE_TEMPERATURE 0x2e
#define BMP085_CMD_MEASURE_PRESSURE_O0 0x34 // no oversampling
#define BMP085_CMD_MEASURE_PRESSURE_O1 0x74 // 2x oversampling
#define BMP085_CMD_MEASURE_PRESSURE_O2 0xb4 // 4x oversampling
#define BMP085_CMD_MEASURE_PRESSURE_O3 0xf4 // 8x oversampling

/* result registers */
#define BMP085_RESULT_REG_MSB  0xf6
#define BMP085_RESULT_REG_LSB  0xf7
#define BMP085_RESULT_REG_XLSB 0xf8

uint32_t bmp085_set_reg(uint8_t reg_addr, uint8_t reg_val);
uint8_t bmp085_get_byte_reg(uint8_t reg_addr);
uint16_t bmp085_get_word_reg(uint8_t reg_addr);
void bmp085_get_calibration_coefficients();
uint8_t bmp085_get_temperature_and_pressure(int32_t *temperature, int32_t *pressure);

/* calibration coefficients */
int16_t AC1;
int16_t AC2;
int16_t AC3;
uint16_t AC4;
uint16_t AC5;
uint16_t AC6;
int16_t B1;
int16_t B2;
// int16_t MB;
int16_t MC;
int16_t MD;

void ram(void) {
	/* results */
	int32_t T, p;

	/* clear LCD */
	lcdClear();

	/* init I²C */
	i2cInit(I2CMASTER); // Init I2C

	/* get calibration coefficients */
	bmp085_get_calibration_coefficients();

	/* main loop */
	while (getInputRaw() != BTN_ENTER) {
		/* get temperature & pressure */
		bmp085_get_temperature_and_pressure(&T, &p);
		
		/* clear LCD */
		lcdClear();

		/* display temperature */
		// lcdPrintln("temperature: ");
		lcdPrintInt(T/10);
		lcdPrint(".");
		lcdPrintInt(T%10);
		lcdPrintln(" degC");

		/* display pressure */
		// lcdPrintln("pressure: ");
		lcdPrint(IntToStr(p, 6, 0));
		lcdPrintln(" Pa");
		lcdRefresh();
	}

	return;
}

uint32_t bmp085_set_reg(uint8_t reg_addr, uint8_t reg_val) {
	I2CMasterBuffer[0] = BMP085_I2C_WRITE;
	I2CMasterBuffer[1] = reg_addr;
	I2CMasterBuffer[2] = reg_val;
	I2CWriteLength = 3;
	I2CReadLength = 0;
	return i2cEngine();
}

uint8_t bmp085_get_byte_reg(uint8_t reg_addr) {
	I2CMasterBuffer[0] = BMP085_I2C_WRITE;
	I2CMasterBuffer[1] = reg_addr;
	I2CWriteLength = 2;
	I2CReadLength = 0;
	i2cEngine();
	I2CMasterBuffer[0] = BMP085_I2C_READ;
	I2CWriteLength = 1;
	I2CReadLength = 1;
	i2cEngine();
	return I2CSlaveBuffer[0];
}

uint16_t bmp085_get_word_reg(uint8_t reg_addr) {
	I2CMasterBuffer[0] = BMP085_I2C_WRITE;
	I2CMasterBuffer[1] = reg_addr;
	I2CWriteLength = 2;
	I2CReadLength = 0;
	i2cEngine();
	I2CMasterBuffer[0] = BMP085_I2C_READ;
	I2CWriteLength = 1;
	I2CReadLength = 2;
	i2cEngine();
	return (I2CSlaveBuffer[0]<<8) + (I2CSlaveBuffer[1]);
}


void bmp085_get_calibration_coefficients() {
	/* get coefficients */
	AC1 = bmp085_get_word_reg(0xaa);
	AC2 = bmp085_get_word_reg(0xac);
	AC3 = bmp085_get_word_reg(0xae);
	AC4 = bmp085_get_word_reg(0xb0);
	AC5 = bmp085_get_word_reg(0xb2);
	AC6 = bmp085_get_word_reg(0xb4);
	B1 = bmp085_get_word_reg(0xb6);
	B2 = bmp085_get_word_reg(0xb8);
	// MB = bmp085_get_word_reg(0xba);
	MC = bmp085_get_word_reg(0xbc);
	MD = bmp085_get_word_reg(0xbe);

	/* debug */

	/*
	lcdPrint("AC1: "); lcdPrintInt(AC1); lcdPrintln("");
	lcdPrint("AC2: "); lcdPrintInt(AC2); lcdPrintln("");
	lcdPrint("AC3: "); lcdPrintInt(AC3); lcdPrintln("");
	lcdPrint("AC4: "); lcdPrintInt(AC4); lcdPrintln("");
	lcdPrint("AC5: "); lcdPrintInt(AC5); lcdPrintln("");
	lcdPrint("AC6: "); lcdPrintInt(AC6); lcdPrintln("");
	lcdPrint("B1: "); lcdPrintInt(B1); lcdPrintln("");
	lcdPrint("B2: "); lcdPrintInt(B2); lcdPrintln("");
	lcdPrint("MB: "); lcdPrintInt(MB); lcdPrintln("");
	lcdPrint("MC: "); lcdPrintInt(MC); lcdPrintln("");
	lcdPrint("MD: "); lcdPrintInt(MD); lcdPrintln("");
	lcdRefresh();
	*/

	return;
}

uint8_t bmp085_get_temperature_and_pressure(int32_t *temperature, int32_t *pressure) {
	/* variables */
	int32_t UT;
	int32_t UP;
	int32_t X1, X2, X3;
	int32_t B3, B5, B6;
	uint32_t B4, B7;
	int16_t oss;

	/* results */
	uint8_t msb, lsb, xlsb;
	int32_t T, p;

	/* request temperature measurement */
	bmp085_set_reg(BMP085_CONTROL_REG_ADDR, BMP085_CMD_MEASURE_TEMPERATURE);
	delayms(10);

	/* get response */
	UT = bmp085_get_word_reg(BMP085_RESULT_REG_MSB);

	/* calculate temperature */
	X1 = (UT-AC6) * AC5 / (1 << 15);
	X2 = MC * (1<<11) / (X1+MD);
	B5 = X1+X2;
	T  = (B5+8)/ (1<<4);

	/* request pressure measurement */
	oss = 3;
	bmp085_set_reg(BMP085_CONTROL_REG_ADDR, BMP085_CMD_MEASURE_PRESSURE_O3);

	/* wait for measurement to complete */
	delayms(10);

	/* get response */
	msb	= bmp085_get_byte_reg(BMP085_RESULT_REG_MSB);
	lsb	= bmp085_get_byte_reg(BMP085_RESULT_REG_LSB);
	xlsb = bmp085_get_byte_reg(BMP085_RESULT_REG_XLSB);
	UP = ((msb<<16) + (lsb<<8) + xlsb) >> (8-oss);

	/* calculate pressure */
	B6 = B5 - 4000;
	X1 = (B2 * ((B6*B6) >> 12)) >> 11;
	X2 = AC2 * B6 >> 11;
	X3 = X1 + X2;
	B3 = (((AC1*4+X3) << oss) + 2) / 4;
	X1 = AC3 * B6 >> 13;
	X2 = (B1 * ((B6*B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = AC4 * (X3 + 32768) >> 15;
	B7 = (UP - B3) * (50000 >> oss);
	B7 &= 0xffffffff;
	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}
	X1 = (p>>8) * (p>>8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;
	p = p + ((X1 + X2 + 3791) >> 4);

	/* assgin to output variables */
	*temperature = T;
	*pressure = p;

	return 0;
}
