/*!
 * \file i2c_scan.c
 * \brief I²C scanner for r0ket.
 *
 * \author Andreas Müller <am@0x7.ch>
 * \date 1.8.2013
 * \version 0.1
 *
 * Note: only scans write addresses.
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

uint32_t i2c_scan_addr(uint8_t addr);

void ram(void) {
	uint8_t addr;

	i2cInit(I2CMASTER); // Init I2C

	lcdClear();
	lcdPrintln("hit button ...");
	lcdRefresh();
	while (getInputRaw() != BTN_ENTER);

	lcdClear();
	lcdPrintln("scanning ...");
	lcdRefresh();

	for (addr=0; addr<0xff; addr++) {
		if (addr % 2 == 1) {
			continue;
		}
		if (i2c_scan_addr(addr) == I2CSTATE_ACK) {
			lcdPrint("0x");
			lcdPrint(IntToStrX(addr, 2));
			lcdPrintln(": ACK");
			lcdRefresh();
		}

	}

	delayms(100);
	lcdRefresh();
	while (getInputRaw() != BTN_ENTER);

	return;
}

uint32_t i2c_scan_addr(uint8_t addr) {
	I2CMasterBuffer[0] = addr;
	I2CWriteLength = 1;
	I2CReadLength = 0;
	return i2cEngine();
}
