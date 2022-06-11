#include "Spectrum.h"

#include "AS7341_values.h"  // sensor specific calibration matrix
#include <cmath>
#include "AS7341.h"

void Spectrum::countsToXYZ(double correctionMatrix[][10], double countMatrix[][1], double &X, double &Y, double &Z)
{
    double XYZ[3][1];
    multiplyMatrices(calibrationMatrix, countMatrix, XYZ);

    X = XYZ[0][0];
    Y = XYZ[1][0];
    Z = XYZ[2][0];
}

// X=0.499541, Y=0.361878, Z=0.105754
//
// https://www.waveformlighting.com/tech/calculate-color-temperature-cct-from-cie-1931-xy-coordinates/
// CCT = 1872K
// Duv = -0.0173
void Spectrum::spectrumToXYZ(double cie1931[][3], double spectralData[], double &X, double &Y, double &Z)
{
    double Ysum(0);

    for (int i = 0; i < 401; i++)
    {
        X += cie1931[i][0] * spectralData[i];
        Y += cie1931[i][1] * spectralData[i];
        Z += cie1931[i][2] * spectralData[i];
        Ysum += cie1931[i][1];
    }

    X *= 1 / Ysum;
    Y *= 1 / Ysum;
    Z *= 1 / Ysum;
}

// void Spectrum::spectrum_to_xyz()
//{
//	//for (i = 0, lambda = 380; lambda < 780.1; i++, lambda++)
//	//{
//	//	double Me;
//
//	//	Me = (*spec_intens)(lambda);
//	//	X += Me * cie1931[i][0];
//	//	Y += Me * cie1931[i][1];
//	//	Z += Me * cie1931[i][2];
//	//}
//
//	//XYZsum = X + Y + Z;
//	//*x = X / XYZsum;
//	//*y = Y / XYZsum;
//	//*z = Z / XYZsum;
// }

uint16_t Spectrum::CIE1931_xy_to_CCT(double x, double y)
{
    // See below for the formula used to approximate CCT using CIE 1931 xy values :
    // McCamy's approximation
    // from AMS excel. Note these number vary slightly from wikipedia... don't know why yet
    // CCT(x,y)=-449n^{3}+3525n^{2}-6823.3n+5520.33

    double n = (x - 0.3320) / (0.1858 - y);
    uint16_t cct = static_cast<uint16_t>(437 * pow(n, 3) + 3601 * pow(n, 2) + 6861 * n + 5517);

    return cct;
}

// from waveformlighting.com
double Spectrum::CIE1931_xy_to_duv(double x, double y)
{
    // AMS
    // double u = 4 * X / (X + 15 * Y + 3 * Z);
    // double v = 9 * X / (X + 15 * Y + 3 * Z);

    // waveformlighting
    // I have no idea what all these magic numbers mean...
    double u = (4 * x) / (-2 * x + 12 * y + 3);
    double v = (6 * y) / (-2 * x + 12 * y + 3);

    const double k6 = -0.00616793;
    const double k5 = 0.0893944;
    const double k4 = -0.5179722;
    const double k3 = 1.5317403;
    const double k2 = -2.4243787;
    const double k1 = 1.925865;
    const double k0 = -0.471106;

    double Lfp = sqrt(pow((u - 0.292), 2) + pow((v - 0.24), 2));
    double a = acos((u - 0.292) / Lfp);
    double Lbb = k6 * pow(a, 6) + k5 * pow(a, 5) + k4 * pow(a, 4) + k3 * pow(a, 3) + k2 * pow(a, 2) + k1 * a + k0;

    return Lfp - Lbb;
}

// excel, photometric results
// correct sensor data (gain corrected * factor - offset)) 1x10 matrix
//
//		Fn
// X	n
// Y	n
// Z	n
void Spectrum::multiplyMatrices(double correctionMatrix[][10], double countMatrix[][1], double product[][1])
{
    // columns of matrix A must be equal to rows of matrix B
    int rowA = 3, colA = CHANNEL_F8;
    int rowB = CHANNEL_F8, colB = 1;
    int i, j, k;

    // init
    for (i = 0; i < rowA; i++)
    {
        for (j = 0; j < colB; j++)
        {
            product[i][j] = 0;
        }
    }

    // multiply
    for (i = 0; i < rowA; i++)
    {
        for (j = 0; j < colB; j++)
        {
            for (k = 0; k < colA; k++)
            {
                product[i][j] += correctionMatrix[i][k] * countMatrix[k][j];
            }
        }
    }
}
