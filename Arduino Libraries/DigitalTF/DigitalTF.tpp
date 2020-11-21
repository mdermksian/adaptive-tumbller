/*
 * DigitalTF.tpp
 * Current Version - V0.1
 * Samuel Bednarski
 *
 * See DigitalTF.h for information.
 */

/*
 * void deleteArrays()
 * Helper function to deallocate memory.
 * Inputs:	None
 * Outputs:	None
 */
template<typename I, typename O>
void DigitalTF<I,O>::deleteArrays() {
	delete[] coeff_num;
	delete[] coeff_den;
	delete[] input_hist;
	delete[] output_hist;
}

/*
 * void shiftInput(I curr_input)
 * Helper function to shift array elements right by 1 and set new first element.
 * Inputs:	curr_input - Value to be placed at array front.
 * Outputs:	None
 */
template<typename I, typename O>
void DigitalTF<I,O>::shiftInput(I curr_input) {
	// Shift elements right by 1
	for(size_t i = n_poles; i > 0; --i) {
		input_hist[i] = input_hist[i-1];
	}

	// Store argument as most recent input
	input_hist[0] = curr_input;
}

/*
 * void shiftOutput()
 * Helper function to shift array elements right by 1, leaves first element unchanged.
 * Inputs:	None
 * Outputs:	None
 */
template<typename I, typename O>
void DigitalTF<I,O>::shiftOutput() {
	// Shift elements right by 1
	for(size_t i = n_poles; i > 0; --i) {
		output_hist[i] = output_hist[i-1];
	}
}

/*
 * DigitalTF()
 * Class constructor.
 */
template<typename I, typename O>
DigitalTF<I,O>::DigitalTF()
		: coeff_num{nullptr}, coeff_den{nullptr},
		  n_zeros{0}, n_poles{0},
		  lower_bound{0}, upper_bound{0},
		  bound_set{false},
		  input_hist{nullptr}, output_hist{nullptr} {}

/*
* ~DigitalTF()
* Class destructor.
*/
template<typename I, typename O>
DigitalTF<I,O>::~DigitalTF() {
	deleteArrays();
}

/*
 * int init(float coeff_B[], size_t size_B, float coeff_A[], size_t size_A)
 * Initialization function to set the filter coefficients.
 * Inputs:	coeff_B - Numerator coefficients.
 * 			size_B - Length of numerator array.
 * 			coeff_A - Denominator coefficients.
 *			size_A - Length of denominator array.
 * Outputs:	(0 - causal, -1 noncausal)
 */
template<typename I, typename O>
int DigitalTF<I,O>::init(float coeff_B[], size_t size_B, float coeff_A[], size_t size_A) {
	// Return invalid if noncausal.
	if(size_B > size_A) {
		return -1;
	}

	// Free memory if previously allocated
	deleteArrays();

	// Initialize history arrays
	input_hist = new I[size_A]{0};
	output_hist = new O[size_A]{0};

	// Create TF numerator
	coeff_num = new float[size_B];
	n_zeros = --size_B;
	for(; size_B <= n_zeros; --size_B) {
		coeff_num[size_B] = coeff_B[size_B] / coeff_A[0];
	}

	// Create TF denominator
	coeff_den = new float[size_A];
	n_poles = --size_A;
	for(; size_A <= n_poles; --size_A) {
		coeff_den[size_A] = coeff_A[size_A] / coeff_A[0];
	}

	// Specify not to saturate
	bound_set = false;

	return 0;
}

/*
 * int init(float coeff_B[], size_t size_B, float coeff_A[], size_t size_A, O lower, O upper)
 * Initialization function to set the filter coefficients and saturation bounds.
 * Inputs:	coeff_B - Numerator coefficients.
 * 			size_B - Length of numerator array.
 * 			coeff_A - Denominator coefficients.
 *			size_A - Length of denominator array.
 *			lower - Saturation lower bound.
 *			upper - Saturation upper bound.
 * Outputs:	(0 - causal, -1 noncausal)
 */
template<typename I, typename O>
int DigitalTF<I,O>::init(float coeff_B[], size_t size_B, float coeff_A[], size_t size_A, O lower, O upper) {
	int valid = init(coeff_B, size_B, coeff_A, size_A);
	setBounds(lower, upper);
	return valid;
}

/*
 * void setBounds(O lower, O upper)
 * Function to set the saturation bounds.
 * Inputs:	lower - Saturation lower bound.
 *			upper - Saturation upper bound.
 * Outputs:	None
 */
template<typename I, typename O>
void DigitalTF<I,O>::setBounds(O lower, O upper) {
	// Store bounds
	lower_bound = lower;
	upper_bound = upper;

	// Specify to saturate
	bound_set = true;
}

/*
 * O update(I input)
 * Function to update the TF states and produce a new output. Assumes output is integer.
 * Inputs:	input - Current TF input.
 * Outputs:	output_hist[0] - Next TF output.
 */
template<typename I, typename O>
O DigitalTF<I,O>::update(I input) {
	// Shift history arrays for time step
	shiftInput(input);
	shiftOutput();

	// Initialize output
	float next_out = 0.0;

	// Process input history
	for(size_t j = 0; j <= n_zeros; ++j) {
		next_out += coeff_num[j] * (float) input_hist[n_poles-n_zeros+j];
	}

	// Process output history
	for(size_t j = 1; j <= n_poles; ++j) {
		next_out -= coeff_den[j] * (float) output_hist[j];
	}

	// Save output as output type and round
	output_hist[0] = (O) (next_out + SIGN(next_out) * 0.5);

	// Saturate output (if bounds are defined)
	if(bound_set) {
		if(output_hist[0] < lower_bound) {
			output_hist[0] = lower_bound;
		} else if(output_hist[0] > upper_bound) {
			output_hist[0] = upper_bound;
		}
	}

	return output_hist[0];
}

/*
 * O updateFloat(I input)
 * Function to update the TF states and produce a new output. Assumes output is floating-point.
 * Inputs:	input - Current TF input.
 * Outputs:	output_hist[0] - Next TF output.
 */
template<typename I, typename O>
O DigitalTF<I,O>::updateFloat(I input) {
	// Shift history arrays for time step
	shiftInput(input);
	shiftOutput();

	// Initialize output
	float next_out = 0.0;

	// Process input history
	for(size_t j = 0; j <= n_zeros; ++j) {
		next_out += coeff_num[j] * (float) input_hist[n_poles-n_zeros+j];
	}

	// Process output history
	for(size_t j = 1; j <= n_poles; ++j) {
		next_out -= coeff_den[j] * (float) output_hist[j];
	}

	// Save output as output type
	output_hist[0] = (O) next_out;

	// Saturate output (if bounds are defined)
	if(bound_set) {
		if(output_hist[0] < lower_bound) {
			output_hist[0] = lower_bound;
		} else if(output_hist[0] > upper_bound) {
			output_hist[0] = upper_bound;
		}
	}

	return output_hist[0];
}

/*
 * O getOutput()
 * Getter function to retrieve the most recent output.
 * Inputs:	None
 * Outputs:	None
 */
template<typename I, typename O>
O DigitalTF<I,O>::getOutput() {
	return output_hist[0];
}
