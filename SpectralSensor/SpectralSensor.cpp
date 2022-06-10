
#include "AS7341.h"
#include "AS7341_values.h"
#include "MCP2221.h"
#include "Spectrum.h"
#include <iostream>
#include <mcp2221_dll_um.h>

void checkChannelDataCalcs()
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
    printf("raw = %d \t counts = %f \t corr = %f", raw, basicCount, corrCount);
}

/* Photometric Results after XYZ Calibration(from xls)
* "Simplest Calculation Method:
This table is based on the (small) XYZ Calibration Matrix in the ""Correction
Values"" Sheet, data from the ""Standards"" Sheet tables, and Sensor Data
(Corr) Channel Data - Ref Column E on this page"

source XML v3.0
X	0.13498
Y	0.14594
Z	0.17119

x	0.29856
y	0.32279
z	0.37865

CCT 7413K
*/
void checkCIE1931Calcs()
{
    double XYZ[3][1];
    Spectrum::multiplyMatrices(calibrationMatrix, tstCorrectedCounts, XYZ);

    double X = XYZ[0][0];
    double Y = XYZ[1][0];
    double Z = XYZ[2][0];

    printf("X = %f, Y = %f, Z = %f\n", X, Y, Z);

    double XYZsum = (X + Y + Z);
    double x = X / XYZsum;
    double y = Y / XYZsum;
    double z = Z / XYZsum;

    printf("x = %f, y = %f, z = %f\n", x, y, z);

    // CCT and duv
    uint16_t cct = Spectrum::CIE1931_xy_to_CCT(x, y);
    double duv = Spectrum::CIE1931_xy_to_duv(x, y);
    printf("CCT = %dK, duv = %f\n", cct, duv);
}

// For a spectral reconstruction, a correction matrix is required, which converts the sensor signals with all
// the filters(e.g. 8 x VIS and NIR + CLEAR = > 10) into a reconstructed spectrum according to the step
// width(n) of the targets.
void spectralReconstruction()
{
    // TODO

    // double resultMatrix[400][10];
    //  {S380nm} = spectralCorrectionMatrix{cor 380nm F1 ... cor 380nm Fn} * {ch F1(t)}
}

int main()
{
    printf("SpectralSensor");

    //*********DEBUG***********
    // checkChannelDataCalcs();
    // checkCIE1931Calcs();
    //
    // exit(-50);
    //********************

    MCP2221 i2cController(AS7341_I2CADDR_DEFAULT);
    i2cController.connect();
    i2cController.printInfo();

    AS7341 sensor(i2cController);
    if (!sensor.init())
    {
        exit(6);
    }

    // get sensor data
    // recommended: atime 29, astep 599 -> 50ms -> 0.083400???
    // TINT 182 ->
    sensor.setATIME(29);  // recommended 29
    sensor.setASTEP(599); // 2,87 us steps, default 999 (2,78 ms), recommended 599
    // uint16_t astep = sensor.getASTEP();
    // exit(100);
    sensor.setGain(AS7341_GAIN_32X);
    sensor.setAutoGain(false);

    // sensor.setLEDCurrent(15);
    //  sensor.enableLED(true);

    sensor.readAllChannels();

    // sensor.enableLED(false);

    sensor.calculateBasicCounts();
    sensor.applyGainCorrection(tstGainCorrections);
    sensor.calculateDataSensorCorrection();

    // apply corrections
    double correctedCounts[10];
    sensor.getCorrectedCounts(correctedCounts);

    std::cout << std::endl;
    std::cout << "F1 415nm (violet): " << sensor.getChannel(AS7341_CHANNEL_F1) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F1) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F1)
              << std::endl;
    std::cout << "F2 445nm (blue)  : " << sensor.getChannel(AS7341_CHANNEL_F2) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F2) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F2)
              << std::endl;
    std::cout << "F3 480nm (l blue): " << sensor.getChannel(AS7341_CHANNEL_F3) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F3) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F3)
              << std::endl;
    std::cout << "F4 515nm (azure) : " << sensor.getChannel(AS7341_CHANNEL_F4) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F4) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F4)
              << std::endl;
    std::cout << "F5 555nm (green) : " << sensor.getChannel(AS7341_CHANNEL_F5) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F5) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F5)
              << std::endl;
    std::cout << "F6 590nm (yellow): " << sensor.getChannel(AS7341_CHANNEL_F6) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F6) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F6)
              << std::endl;
    std::cout << "F7 630nm (amber) : " << sensor.getChannel(AS7341_CHANNEL_F7) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F7) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F7)
              << std::endl;
    std::cout << "F8 680nm (red)   : " << sensor.getChannel(AS7341_CHANNEL_F8) << "\t"
              << sensor.getBasicCount(AS7341_CHANNEL_F8) << "\t" << sensor.getCorrectedCount(AS7341_CHANNEL_F8)
              << std::endl;
    std::cout << "Clear            : " << sensor.getChannel(AS7341_CHANNEL_CLEAR) << std::endl;
    std::cout << "Near IR          : " << sensor.getChannel(AS7341_CHANNEL_NIR) << std::endl;
    std::cout << std::endl;

    // put in matrix
    double countMatrix[10][1];
    for (int i = 0; i < 10; i++)
    {
        countMatrix[i][0] = correctedCounts[i];
    }

    // get XYZ
    double X(0), Y(0), Z(0);
    Spectrum::countsToXYZ(calibrationMatrix, countMatrix, X, Y, Z);
    printf("X = %f, Y = %f, Z = %f\n", X, Y, Z);

    double x(0), y(0);
    double XYZsum = X + Y + Z;
    x = X / XYZsum;
    y = Y / XYZsum;

    // CCT and duv
    uint16_t cct = Spectrum::CIE1931_xy_to_CCT(x, y);
    double duv = Spectrum::CIE1931_xy_to_duv(x, y);
    printf("CCT = %dK, Duv = %f\n", cct, duv);
}
