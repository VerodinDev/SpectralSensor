#include "Spectrum.h"

#include "AS7341_values.h"  // sensor specific calibration matrix
#include <cmath>
#include "AS7341.h"
#include <fstream>

using namespace std;

void Spectrum::countsToXYZ(double correctionMatrix[][10], double countMatrix[][1], double &X, double &Y, double &Z)
{
    const uint8_t rows = 3; // xyz values

    double XYZ[rows][1];
    multiplyMatrices(calibrationMatrix_v3, countMatrix, XYZ, rows, 10);

    X = XYZ[0][0];
    Y = XYZ[1][0];
    Z = XYZ[2][0];
}

void Spectrum::spectrumToXYZ(const vector<double>& spd, Tristimulus& XYZ)
{
    uint8_t stepsize = spd.size() > 201 ? 1 : 2;

    for (uint16_t i = 0; i < spd.size(); i++)
    {
        XYZ.X += spd[i] * cie1931[i * stepsize][0];
        XYZ.Y += spd[i] * cie1931[i * stepsize][1];
        XYZ.Z += spd[i] * cie1931[i * stepsize][2];
    }
}

// AMS
void Spectrum::spectrumToXYZ_AMS(double spectralData[][1], Tristimulus& XYZ, const uint16_t wavelengths)
{
    for (uint16_t i = 0; i < wavelengths; i++)
    {
        XYZ.X += spectralData[i][0] * cie1931[i][0];    // TODO stepsize
        XYZ.Y += spectralData[i][0] * cie1931[i][1];
        XYZ.Z += spectralData[i][0] * cie1931[i][2];
    }
}

// X=0.499541, Y=0.361878, Z=0.105754
//
// https://www.waveformlighting.com/tech/calculate-color-temperature-cct-from-cie-1931-xy-coordinates/
// CCT = 1872K
// Duv = -0.0173
//void Spectrum::spectrumToXYZ(double cie1931[][3], double spectralData[], double &X, double &Y, double &Z)
//{
//    double Ysum(0);
//
//    for (int i = 0; i < 401; i++)
//    {
//        X += cie1931[i][0] * spectralData[i];
//        Y += cie1931[i][1] * spectralData[i];
//        Z += cie1931[i][2] * spectralData[i];
//        Ysum += cie1931[i][1];
//    }
//
//    X *= 1 / Ysum;
//    Y *= 1 / Ysum;
//    Z *= 1 / Ysum;
//}

void Spectrum::XYZtoXy(const Tristimulus& XYZ, Chromaticity& xy)
{
    double XYZsum = XYZ.X + XYZ.Y + XYZ.Z;

    xy.x = XYZ.X / XYZsum;
    xy.y = XYZ.Y / XYZsum;
}

uint16_t Spectrum::CIE1931_xy_to_CCT(const Chromaticity& xy)
{
    // See below for the formula used to approximate CCT using CIE 1931 xy values :
    // McCamy's approximation
    // Source: AMS excel
    // Note 1: differs from wikipedia: CCT(x,y)=-449n^{3}+3525n^{2}-6823.3n+5520.33
    // Note 2: waveformlighting.com uses same formula as AMS

    double n = (xy.x - 0.3320) / (0.1858 - xy.y);
    uint16_t cct = static_cast<uint16_t>(437 * pow(n, 3) + 3601 * pow(n, 2) + 6861 * n + 5517);

    return cct;
}

uint16_t Spectrum::CIE1931_xy_to_CCT_wikipedia(const Chromaticity& xy)
{
    double n = (xy.x - 0.325) / (xy.y - 0.154);
    uint16_t cct = static_cast<uint16_t>(-449 * pow(n, 3) + 3525 * pow(n, 2) - 6823.3 * n + 5520.33);

    return cct;
}

// from waveformlighting.com
float Spectrum::CIE1931_xy_to_duv(const Chromaticity& xy)
{
    // AMS
    // double u = 4 * X / (X + 15 * Y + 3 * Z);
    // double v = 9 * X / (X + 15 * Y + 3 * Z);

    // convert chromaticities to CIE 1960
    double u = (4 * xy.x) / (-2 * xy.x + 12 * xy.y + 3);
    double v = (6 * xy.y) / (-2 * xy.x + 12 * xy.y + 3);

    // I have no idea what all these magic numbers mean...
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

    return static_cast<float>(Lfp - Lbb);
}

void Spectrum::reconstructSpectrum(double spectralMatrix[][10], double countMatrix[][1], double reconstructedSpectrum[][1], const uint16_t wavelengths)
{
    multiplyMatrices(spectralMatrix, countMatrix, reconstructedSpectrum, wavelengths, 10);

    //// normalise
    //double highestValue(0);

    //for (uint16_t i = 0; i < wavelengths; i++)
    //{
    //    if (reconstructedSpectrum[i][0] > highestValue)
    //    {
    //        highestValue = reconstructedSpectrum[i][0];
    //    }
    //}
    //
    //for (uint16_t i = 0; i < wavelengths; i++)
    //{
    //    reconstructedSpectrum[i][0] /= highestValue;
    //}
}

void Spectrum::saveToCsv(double spectralData[][1], std::string filename)
{
    uint16_t wavelengths = 780 - 380;

    std::ofstream cvsFile;
    
    cvsFile.open(filename);

    cvsFile << "nm  value" << std::endl;
    for (uint16_t wavelength = 0; wavelength <= wavelengths; wavelength++)
    {
        cvsFile << wavelength + 380 << " " << spectralData[wavelength][0] << std::endl;
    }

    cvsFile.close();
}

// excel, photometric results
// correct sensor data (gain corrected * factor - offset)) 1x10 matrix
//
//		Fn
// X	n
// Y	n
// Z	n
void Spectrum::multiplyMatrices(double correctionMatrix[][10], double countMatrix[][1], double product[][1], uint16_t rows, const uint8_t columns)
{
    // columns of matrix A must be equal to rows of matrix B
    int rowA = rows, colA = columns;
    int rowB = columns, colB = 1;
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
                //printf("i=%d j=%d k=%d\n", i, j, k); OK
            }
        }
    }
}
