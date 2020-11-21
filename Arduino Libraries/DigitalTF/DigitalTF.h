/*
 * DigitalTF.h
 * Current Version - V0.1
 * Samuel Bednarski
 *
 * History:
 * 20200926 - V0.1 completed.
 *
 * The DigitalTF implements an arbitrary-order discrete-time transfer function.
 */

#ifndef DIGITALTF_H_
#define DIGITALTF_H_

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#endif

#define SIGN(n) (n<0 ? -1 : 1)	// sign() macro (-1 if negative, +1 if zero or positive)

template<typename I, typename O>
class DigitalTF {
private:
	float* coeff_num;
	float* coeff_den;
	size_t n_zeros;
	size_t n_poles;
	O lower_bound;
	O upper_bound;
	bool bound_set;
	I* input_hist;
	O* output_hist;

	void deleteArrays();
	void shiftInput(I);
	void shiftOutput();

public:
	DigitalTF();
	~DigitalTF();
	int init(float*, size_t, float*, size_t);
	int init(float*, size_t, float*, size_t, O, O);
	void setBounds(O, O);
	O update(I);
	O updateFloat(I);
	O getOutput();
};

#include "DigitalTF.tpp"

#endif /* DIGITALTF_H_ */
