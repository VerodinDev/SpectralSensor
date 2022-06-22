#pragma once

#include <cstdint>
#include <string>
#include <vector>

const uint16_t ALL_WAVELENGTHS = 1000 - 380 + 1;
const uint16_t VISIBLE_WAVELENGTHS = 780 - 380 + 1;

struct Tristimulus
{
    Tristimulus() : X(0), Y(0), Z(0)
    {
    }

    double X;
    double Y;
    double Z;
};

struct Chromaticity
{
    Chromaticity() : x(0), y(0)
    {
    }

    double x;
    double y;
};

class Spectrum
{
  public:
    typedef std::vector<std::vector<double>> Matrix;

    // AS7341 counts to XYZ
    static void countsToXYZ(const Matrix &correctionMatrix, const std::vector<double> &counts, Tristimulus &XYZ);

    // spectrum to XYZ
    static void spectrumToXYZ(const std::vector<double> &spd, Tristimulus &XYZ);

    // XYZ to xy
    static void XYZtoXy(const Tristimulus &XYZ, Chromaticity &xy);

    // CIE 1931 xy to CCT
    // McCamy's approximation
    static uint16_t CIE1931_xy_to_CCT(const Chromaticity &xy);
    static uint16_t CIE1931_xy_to_CCT_wikipedia(const Chromaticity &xy);

    // CIE 1931 xy to Duv
    static float CIE1931_xy_to_duv(const Chromaticity &xy);

    // spectral reconstruction based on channel data (AMS specific)
    static void reconstructSpectrum(const Matrix &spectralMatrix, const std::vector<double> &countMatrix,
                                    std::vector<double> &reconstructedSpectrum);

    // save to CSV
    static void saveToCsv(const std::vector<double> &spd, const std::string &filename);

    // matrix multiplication
    static void multiplyMatrices(const Matrix &matrixA, const Matrix &matrixB, Matrix &product,
                                 const size_t rows, const size_t columns);

    static void toMatrix(const std::vector<double>& values, Matrix& matrix);

    static void toArray(const Matrix& matrix, std::vector<double>& values);

    static bool isNegative(double v);
};
