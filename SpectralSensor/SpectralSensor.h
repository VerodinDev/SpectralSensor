#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include "ColorRenderingIndex.h"

class MCP2221;
class AS7341;

// enable calc verification against example values from AMS excel
#define VERIFY_CALCS

class SpectralSensor
{
  public:
    SpectralSensor();
    // virtual ~SpectralSensor();

    void setupI2C();
    void setupAS7341();
    void loadCorrectionMatrix();
    void takeReading();

#ifdef VERIFY_CALCS
    void checkChannelDataCalcs();
    void checkCIE1931Calcs(uint8_t lastChannel);
    void verifySpectralReconstruction();
#endif

  private:
    typedef std::vector<std::vector<double>> Matrix;

#ifdef VERIFY_CALCS
    void multiplyMatrices(const Matrix& matrixA, const Matrix& matrixB, Matrix& product, const size_t rows, const size_t columns);

    void toMatrix(const std::vector<double>& values, Matrix& matrix);

    void toArray(const Matrix& matrix, std::vector<double>& values);
#endif

    void readCSV(const std::string &filename, Matrix &correctionMatrix);

    // I2C controller
    MCP2221 *m_pI2cController;

    // spectral sensor
    AS7341 *m_pSensor;

    // correction matrix
    Matrix m_correctionMatrix;

    ColorRenderingIndex m_cri;
};
