#ifndef _CONVERTERS_H_
#define _CONVERTERS_H_

#include "main.h" // IWYU pragma: keep
#include <cmath>
#include <numbers>

inline double cDTI(double degrees) { // Convert Degrees to inches
	return degrees / 47.0127326151;
}

inline double cDTI2(double degrees) {
	return ((double)degrees / 100) * 0.0174329;
}

inline double cITD(double inches) { // Convert inches to degrees
	return inches * 47.0127326151;
}

inline double cITM(double inches) { // Convert inches to meters
	return inches / 39.37;
}

inline double cMTI(double inches) { // Convert inches to meters
	return inches * 39.37;
}

inline double cDTR(double degrees) { // Convert degrees to radians
	return degrees * (std::numbers::pi / 180);
}

inline double cRTD(double radians) { // Convert radians to degrees
	return radians * (180 / std::numbers::pi);
}

inline double to_sec(double time) {
  return time / 1000;
}


#endif // _CONVERTERS_H_