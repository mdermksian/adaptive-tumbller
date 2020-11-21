/*
 * EncoderI2C.h
 * Current Version - V0.1
 * Samuel Bednarski
 *
 * See EncoderI2C.h for information.
 */

#include "EncoderI2C.h"

/*
 * EncoderI2C()
 * Default class constructor.
 */
EncoderI2C::EncoderI2C() : addr{0}, count{0} {
	if(!wire_lib_init) {
		Wire.begin();
		wire_lib_init = true;
	}
}

/*
 * EncoderI2C(uint8_t i2c_addr)
 * Class constructor with I2C address.
 * Inputs:	i2c_addr - 7-bit I2C address
 */
EncoderI2C::EncoderI2C(uint8_t i2c_addr) : addr{i2c_addr}, count{0} {
	if(!wire_lib_init) {
		Wire.begin();
		wire_lib_init = true;
	}
}

/*
 * void setAddr(uint8_t i2c_addr)
 * Set new I2C address.
 * Inputs:	i2c_addr - 7-bit I2C address
 * Outputs:	None
 */
void EncoderI2C::setAddr(uint8_t i2c_addr) {
	addr = i2c_addr;
}

/*
 * uint8_t getAddr()
 * Return encoder I2C address.
 * Inputs:	None
 * Outputs:	7-bit I2C address
 */
uint8_t EncoderI2C::getAddr() {
	return addr;
}

/*
 * int32_t getCount()
 * Polls sensor and returns encoder count.
 * Inputs:	None
 * Outputs:	encoder count
 */
int32_t EncoderI2C::readEncoder() {
	uint8_t num_bytes;
	union new_count_t {
		int32_t num;		// numeric representation of new count
		uint8_t buff[4];	// byte array for accumulation
	} new_count;

	// Flush buffer
	while(Wire.available()) {
		Wire.read();
	}

	// Read 4 bytes from sensor
	num_bytes = Wire.requestFrom(addr, 4);

	// Limit returned bytes to 4
	if(num_bytes > 4) {
		num_bytes = 4;
	}

	// Accumulate bytes into counter
	for(size_t i = 0; i < num_bytes; ++i) {
		new_count.buff[i] = Wire.read();
	}

	// Update private variables
	change = new_count.num - count;	// calculate change
	count = new_count.num;			// save new count

	return count;
}

/*
 * int32_t getCount()
 * Return most recent encoder count. Doesn't poll sensor.
 * Inputs:	None
 * Outputs:	encoder count
 */
int32_t EncoderI2C::getCount() {
	return count;
}

/*
 * int32_t getChange()
 * Return difference between current and previous encoder count. Doesn't poll sensor.
 * Inputs:	None
 * Outputs:	change in encoder count
 */
int32_t EncoderI2C::getChange() {
	return change;
}
