/*
 * EncoderI2C.h
 * Current Version - V0.1
 * Samuel Bednarski
 *
 * History:
 * 20201113 - V0.1 completed.
 *
 * The EncoderI2C reads encoder count data from an I2C device. For Arduino only.
 */

#ifndef ENCODERI2C_H_
#define ENCODERI2C_H_

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif

#include <Wire.h>

class EncoderI2C {
private:
	bool wire_lib_init = false;
	uint8_t addr;
	int32_t count;
	int32_t change;

public:
	EncoderI2C();
	EncoderI2C(uint8_t);
	void setAddr(uint8_t);
	uint8_t getAddr();
	int32_t readEncoder();
	int32_t getCount();
	int32_t getChange();
};

#endif /* ENCODERI2C_H_ */
