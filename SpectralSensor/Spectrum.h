#pragma once

#include <cstdint>
#include <string>
#include <vector>

const uint16_t ALL_WAVELENGTHS = 1000 - 380;    // +1?
const uint16_t VISIBLE_WAVELENGTHS = 780 - 380 + 1;

struct Tristimulus
{
    Tristimulus() : X(0), Y(0), Z(0) {}

    double X;
    double Y;
    double Z;
};

struct Chromaticity
{
    Chromaticity() : x(0), y(0) {}

    double x;
    double y;
};

class Spectrum
{
  public:
    // AS7341 counts to XYZ
    static void countsToXYZ(double correctionMatrix[][10], double countMatrix[][1], double &X, double &Y, double &Z);

    // spectrum to XYZ
    static void spectrumToXYZ(const std::vector<double>& spd, Tristimulus& XYZ);
    static void spectrumToXYZ_AMS(double spectralData[][1], Tristimulus& XYZ, const uint16_t wavelengths);

    // XYZ to xy
    static void XYZtoXy(const Tristimulus& XYZ, Chromaticity& xy);

    // CIE 1931 xy to CCT
    // McCamy's approximation
    static uint16_t CIE1931_xy_to_CCT(const Chromaticity& xy);
    static uint16_t CIE1931_xy_to_CCT_wikipedia(const Chromaticity& xy);

    // CIE 1931 xy to Duv
    static float CIE1931_xy_to_duv(const Chromaticity& xy);

    // spectral reconstruction based on channel data
    static void reconstructSpectrum(double spectralMatrix[][10], double countMatrix[][1], double reconstructedSpectrum[][1], const uint16_t wavelengths = 780 - 380);

    // save to CSV
    static void saveToCsv(double spectralData[][1], std::string filename);

    // matrix multiplication
    static void multiplyMatrices(double matrixA[][10], double matrixB[][1], double product[][1], const uint16_t rows, const uint8_t columns);
};
