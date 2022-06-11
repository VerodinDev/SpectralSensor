
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

void takeReading(AS7341 &sensor)
{
    // sensor.setLEDCurrent(15);
    // sensor.enableLED(true);

    sensor.readAllChannels();

    // sensor.enableLED(false);

    sensor.calculateBasicCounts();
    sensor.applyGainCorrection(tstGainCorrections);
    sensor.calculateDataSensorCorrection();

    // apply corrections
    double correctedCounts[10];
    sensor.getCorrectedCounts(correctedCounts);

    // output channel raw and corrected counts
    printf("\n");
    for (uint8_t channel = CHANNEL_F1; channel < CHANNEL_F8; channel++)
    {
        printf("F%d %d\t %f \n", channel, sensor.getRawValue(static_cast<SpectralChannel>(channel)),
               sensor.getCorrectedCount(static_cast<SpectralChannel>(channel)));
    }
    printf("\n");

    // put in matrix
    double countMatrix[CHANNEL_F8][1] = {0};
    for (int i = 0; i < CHANNEL_F8; i++)
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
    printf("******************\n");
}

int main()
{
    printf("*** Spectral sensor v0.1 ***\n");

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
    //sensor.setAutoGain(true);

    // take n readings
    for (uint8_t readings = 0; readings < 10; readings++)
    {
        takeReading(sensor);
    }
}
