#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#include <cmath>
#include <limits>

namespace cyclone {
	// Define the precision of the floating point numbers
	typedef float real;

	// Define the precision of the square root operator
	#define real_sqrt sqrtf

	// Define the precision of the power operator
	#define real_pow powf
	
	// Define max for reals
	#define REAL_MAX DBL_MAX
	
	// Define the precision of the absolute magnitude operator
	#define real_abs fabsf
}

#endif// CYCLONE_PRECISION_H