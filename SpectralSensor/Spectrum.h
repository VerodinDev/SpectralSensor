#pragma once

#include <cstdint>

class Spectrum
{
public:

	// 
	static void countsToXYZ(double correctionMatrix[][10], double countMatrix[][1], double& X, double& Y, double& Z);

	//
	static void spectrumToXYZ(double cie1931[][3], double spectralData[], double& X, double& Y, double& Z);

	// CIE 1931 xy to Duv
	static double CIE1931_xy_to_duv(double x, double y);

	// CIE 1931 xy to CCT
	// McCamy's approximation
	static uint16_t CIE1931_xy_to_CCT(double x, double y);

//private:

	static void multiplyMatrices(double matrixA[][10], double matrixB[][1], double product[][1]);
};
