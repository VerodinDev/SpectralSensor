
#include "SpectralSensor.h"
#include "AS7341.h"
#include "AS7341_values.h"
#include "MCP2221.h"
#include "Spectrum.h"
//#include <iostream>
#include <mcp2221_dll_um.h>
//#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std;

// uncomment to use XYZ calibration matrix. Otherwise spectral calibration matrix is used
//#define USE_XYZ_CALIBRATION_MATRIX

SpectralSensor::SpectralSensor() : m_pI2cController(0), m_pSensor(0)
{
}

void SpectralSensor::setupI2C()
{
    m_pI2cController = new MCP2221(AS7341_I2CADDR_DEFAULT);
    m_pI2cController->connect();
    m_pI2cController->printInfo();
}

void SpectralSensor::setupAS7341()
{
    m_pSensor = new AS7341(*m_pI2cController);
    m_pSensor->init();

    // get sensor data
    // recommended: atime 29, astep 599 -> 50ms -> 0.083400???
    // TINT 182 ->
    m_pSensor->setATIME(29);  // recommended 29
    m_pSensor->setASTEP(599); // 2,87 us steps, default 999 (2,78 ms), recommended 599
    m_pSensor->setGain(AS7341_GAIN_32X);
    // m_sensor->setAutoGain(true);

    printf("Init done\n");
#ifdef USE_XYZ_CALIBRATION_MATRIX
    printf("Using XYZ calibration matrix\n\n");
#else
    printf("Using spectral calibration matrix\n\n");
#endif
}

void SpectralSensor::loadCorrectionMatrix()
{
#ifdef USE_XYZ_CALIBRATION_MATRIX
    string csvFile = "../data/XYZCorrectionMatrix_v3.0.0.csv";
    uint16_t rows = 3;
#else
    string csvFile = "../data/SpectralCorrectionMatrix_v3.0.csv";
    uint16_t rows = ALL_WAVELENGTHS;
#endif

    // create correction matrix
    uint8_t channels = 10;
    m_correctionMatrix.resize(rows);
    for (uint16_t i = 0; i < ALL_WAVELENGTHS; i++)
    {
        m_correctionMatrix[i].resize(channels);
    }

    readCSV(csvFile, m_correctionMatrix);

    // load TCS table
    m_cri.loadTCSTable();
}

void SpectralSensor::takeReading()
{
    // sensor.setLEDCurrent(15);
    // sensor.enableLED(true);

    m_pSensor->readAllChannels();

    // sensor.enableLED(false);

    m_pSensor->calculateBasicCounts();
    m_pSensor->applyGainCorrection(tstGainCorrections_v3);
    m_pSensor->calculateDataSensorCorrection();

    // apply corrections
    double correctedCounts[10];
    m_pSensor->getCorrectedCounts(correctedCounts);

    // output channel raw and corrected counts
    printf("\n");
    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_F8; channel++)
    {
        printf("F%d\t%d\t %f \n", channel + 1, m_pSensor->getRawValue(static_cast<SpectralChannel>(channel)),
               m_pSensor->getCorrectedCount(static_cast<SpectralChannel>(channel)));
    }
    printf("Clear\t%d\t %f \n", m_pSensor->getRawValue(CHANNEL_CLEAR), m_pSensor->getCorrectedCount(CHANNEL_CLEAR));
    printf("NIR\t%d\t %f \n", m_pSensor->getRawValue(CHANNEL_NIR), m_pSensor->getCorrectedCount(CHANNEL_NIR));
    printf("\n");

    // array to vector
    // TODO
    vector<double> correctedCountsV(10);
    for (int i = 0; i < 10; i++)
    {
        correctedCountsV[i] = correctedCounts[i];
    }

    // get XYZ
    Tristimulus XYZ;

#ifdef USE_XYZ_CALIBRATION_MATRIX

    Spectrum::countsToXYZ(m_correctionMatrix, correctedCounts, XYZ);

#else // use spectral calibration matrix

    // create spectral correction matrix
    uint8_t channels = 10;

    // reconstruct spectrum and get XYZ values
    vector<double> reconstructedSpd;
    reconstructedSpd.resize(ALL_WAVELENGTHS);

    m_pSensor->reconstructSpectrum(m_correctionMatrix, correctedCountsV, reconstructedSpd);

    Spectrum reconstructedSpectrum(reconstructedSpd);
    reconstructedSpectrum.saveToCsv("reconstructedSpectrum.csv");

    reconstructedSpectrum.getXYZ(XYZ);

#endif

    printf("X = %f, Y = %f, Z = %f\n", XYZ.X, XYZ.Y, XYZ.Z);

    // get xy
    Chromaticity xy;
    reconstructedSpectrum.XYZtoXy(XYZ, xy);
    printf("x = %f, y = %f\n", xy.x, xy.y);

    // CCT and duv
    uint16_t cct = reconstructedSpectrum.xyToCCT(xy);
    uint16_t cct2 = reconstructedSpectrum.xyToCCT_wikipedia(xy); // for debug comparison purposes only
    double duv = reconstructedSpectrum.xyToDuv(xy);
    printf("CCT = %dK, Duv = %f\n", cct, duv);
    printf("CCT = %dK (wikipedia)\n", cct2);

    // CRI
    uint8_t Ri[MAX_TCS];
    m_cri.calculateCRI(reconstructedSpectrum, Ri);

    for (uint8_t i = 0; i < MAX_TCS; i++)
    {
        printf("R%d = %d ", i + 1, Ri[i]);
    }
    printf("\n");

    printf("******************\n");
}

#ifdef VERIFY_CALCS

bool compareDouble(const double& a, const double& b, double epsilon = 0.0000001f)
{
    return (fabs(a - b) < epsilon);
}

// copy of AS7341::multiplyMatrices
void SpectralSensor::multiplyMatrices(const Matrix &matrixA, const Matrix &matrixB, Matrix &product, const size_t rows,
                                      const size_t columns)
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
                // printf("i=%d j=%d k=%d\n", i, j, k); OK
            }
        }
    }
}

void SpectralSensor::toMatrix(const vector<double> &values, Matrix &matrix)
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

void SpectralSensor::toArray(const Matrix &matrix, vector<double> &values)
{
    for (uint16_t i = 0; i < matrix.size(); i++)
    {
        values[i] = matrix[i][0];
    }
}

void SpectralSensor::checkChannelDataCalcs()
{
    uint16_t raw = 1236;
    float gain_val = 512;

    // recommended 999 and 29 -> 50ms
    uint16_t astep = 599; // default 999
    uint8_t atime = 29;   // default 0

    // float  50.0400009
    // double 50.039999999999999
    double tint = (atime + 1) * (astep + 1) * 2.78 / 1000;

    // double basicCount = (raw / (gain_val * tint));
    double basicCount = tstBasicCounts[0];
    double corrCount = /*correctionFactors[0]*/ 1.02811245 * (basicCount - /*offsetCompensationValues[0]*/ 0.00196979);
    printf("raw = %d \t counts = %f \t corr = %f\n", raw, basicCount, corrCount);
}

void SpectralSensor::checkCIE1931Calcs(uint8_t noOfChannels)
{
    printf("\n*** check CIE1931 calcs (%d channels) ***\n", noOfChannels);

    Matrix correctedCountsMatrix;
    toMatrix(tstCorrectedCounts, correctedCountsMatrix);

    vector<double> dummyXYZ(3);
    Matrix XYZmatrix;
    toMatrix(dummyXYZ, XYZmatrix);
    multiplyMatrices(m_correctionMatrix, correctedCountsMatrix, XYZmatrix, 3, noOfChannels);

    Tristimulus XYZ;
    XYZ.X = XYZmatrix[0][0];
    XYZ.Y = XYZmatrix[1][0];
    XYZ.Z = XYZmatrix[2][0];

    printf("X = %f, Y = %f, Z = %f\n", XYZ.X, XYZ.Y, XYZ.Z);

    Spectrum dummySpd;

    // get xy
    Chromaticity xy;
    dummySpd.XYZtoXy(XYZ, xy);
    printf("x = %f, y = %f\n", xy.x, xy.y);

    // CCT and duv
    uint16_t cct = dummySpd.xyToCCT(xy);
    double duv = dummySpd.xyToDuv(xy);
    printf("CCT = %dK, duv = %f\n", cct, duv);
}

void SpectralSensor::verifySpectralReconstruction()
{
    printf("\n*** verify spectral reconstruction ***\n");

    const uint16_t wavelengths = ALL_WAVELENGTHS;
    const uint16_t visibleWavelengths = VISIBLE_WAVELENGTHS;

    MCP2221 dummyDev(0x00);
    AS7341 sensor(dummyDev);
    vector<double> reconstructedSpectrum;

    reconstructedSpectrum.resize(ALL_WAVELENGTHS);
    sensor.reconstructSpectrum(m_correctionMatrix, tstCorrectedCounts, reconstructedSpectrum);

    Spectrum reconstructedSpd(reconstructedSpectrum);

    // dump 1st 20 values to screen
    for (uint16_t wavelength = 0; wavelength < 20; wavelength++)
    {
        printf("Wavelength %d = %f\n", wavelength + 380, reconstructedSpectrum[wavelength]);
    }

    // spectrum to XYZ
    Tristimulus XYZ;
    reconstructedSpd.getXYZ(XYZ);
    printf("X = %f, Y = %f, Z = %f\n", XYZ.X, XYZ.Y, XYZ.Z);

    //
    reconstructedSpd.saveToCsv("reconstructedSpectrum.csv");

    // get xy
    Chromaticity xy;
    reconstructedSpd.XYZtoXy(XYZ, xy);
    printf("x = %f, y = %f\n", xy.x, xy.y);

    // CCT and duv
    uint16_t cct = reconstructedSpd.xyToCCT(xy);
    double duv = reconstructedSpd.xyToDuv(xy);
    printf("CCT = %dK, duv = %f\n", cct, duv);
}

#endif

void SpectralSensor::readCSV(const string &filename, Matrix &correctionMatrix)
{
    ifstream csvFile;

    csvFile.open(filename, ifstream::in);
    if (!csvFile.is_open())
    {
        throw runtime_error("Error opening data file " + filename);
    }

    string line;
    uint16_t row(0);

    // skip header
    getline(csvFile, line);

    // read values
    // 380 0.2190 0.0700 0.0650 0.0740 0.2950 0.1500 0.3780 0.1040 0.0660 0.0500 0.1110 0.1200 0.1040 0.0360
    while (getline(csvFile, line))
    {
        replace(line.begin(), line.end(), ';', ' ');

        stringstream ss(line);
        float value(0);
        uint8_t column = 0;

        // skip wavelength column
        ss >> value;

        // note that there is NO faulty input data check whatsoever!!!
        while (ss >> value)
        {
            correctionMatrix[row][column] = value;
            column++;
        }

        row++;
    }

    csvFile.close();
}
