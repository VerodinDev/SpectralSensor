#include "Spectrum.h"

#include "AS7341_values.h"  // sensor specific calibration matrix
#include <cmath>
#include "AS7341.h"
#include <fstream>
#include <algorithm>

using namespace std;

void Spectrum::countsToXYZ(const Matrix& correctionMatrix, const vector<double>& counts, Tristimulus& XYZ)
{
    // put counts into matrix
    Matrix countMatrix;
    countMatrix.resize(counts.size());
    for (uint8_t i = 0; i < counts.size(); i++)
    {
        countMatrix[i][0] = counts[i];
    }

    Matrix XYZmatrix;
    XYZmatrix.resize(3);
    multiplyMatrices(correctionMatrix, countMatrix, XYZmatrix, XYZmatrix.size(), counts.size());

    // get XYZ from matrix
    XYZ.X = XYZmatrix[0][0];
    XYZ.Y = XYZmatrix[1][0];
    XYZ.Z = XYZmatrix[2][0];
}

void Spectrum::spectrumToXYZ(const vector<double>& spd, Tristimulus& XYZ)
{
    uint8_t stepsize = spd.size() > 201 ? 1 : 2;
    double Ysum(0);

    for (uint16_t i = 0; i < spd.size(); i++)
    {
        XYZ.X += spd[i] * cie1931[i * stepsize][0];
        XYZ.Y += spd[i] * cie1931[i * stepsize][1];
        XYZ.Z += spd[i] * cie1931[i * stepsize][2];

        Ysum += cie1931[i * stepsize][1];
    }

    // normalise
    XYZ.X *= 1 / Ysum;
    XYZ.Y *= 1 / Ysum;
    XYZ.Z *= 1 / Ysum;
}

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

void Spectrum::reconstructSpectrum(const Matrix& spectralMatrix, const vector<double>& counts, vector<double>& reconstructedSpectrum)
{
    Matrix countMatrix;
    toMatrix(counts, countMatrix);

    // temp matrix to store result
    Matrix reconstructedMatrix;
    reconstructedMatrix.resize(spectralMatrix.size());
    for (uint16_t row = 0; row < spectralMatrix.size(); row++)
    {
        reconstructedMatrix[row].resize(1);
    }

    multiplyMatrices(spectralMatrix, countMatrix, reconstructedMatrix, spectralMatrix.size(), 10);

    toArray(reconstructedMatrix, reconstructedSpectrum);

    // only use visible spectrum from here on
    reconstructedSpectrum.resize(VISIBLE_WAVELENGTHS);

    // strip negative values
    replace_if(reconstructedSpectrum.begin(), reconstructedSpectrum.end(), isNegative, 0);

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

void Spectrum::saveToCsv(const vector<double>& spd, const std::string& filename)
{
    std::ofstream cvsFile;
    
    //__DATE__ __TIME__

    cvsFile.open(filename);

    // header
    //cvsFile << "nm  value" << std::endl;

    // values
    for (uint16_t wavelength = 0; wavelength < spd.size(); wavelength++)
    {
        cvsFile << wavelength + 380 << " " << spd[wavelength] << std::endl;
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
void Spectrum::multiplyMatrices(const Matrix& matrixA, const Matrix& matrixB, Matrix& product, const size_t rows, const size_t columns)
{
    // columns of matrix A must be equal to rows of matrix B
    size_t rowA = rows, colA = columns;
    size_t rowB = columns, colB = 1;
    int i, j, k;

    // multiply
    for (i = 0; i < rowA; i++)
    {
        for (j = 0; j < colB; j++)
        {
            for (k = 0; k < colA; k++)
            {
                product[i][j] += matrixA[i][k] * matrixB[k][j];
                //printf("i=%d j=%d k=%d\n", i, j, k); OK
            }
        }
    }
}

void Spectrum::toMatrix(const vector<double>& values, Matrix& matrix)
{
    // init
    matrix.clear();
    matrix.resize(values.size());

    for (uint16_t row = 0; row < values.size(); row++)
    {
        matrix[row].resize(1);
    }

    // copy values
    for (uint8_t i = 0; i < values.size(); i++)
    {
        matrix[i][0] = values[i];
    }
}

void Spectrum::toArray(const Matrix& matrix, vector<double>& values)
{
    for (uint16_t i = 0; i < matrix.size(); i++)
    {
        values[i] = matrix[i][0];
    }
}

bool Spectrum::isNegative(double v)
{
    return (v < 0);
}
