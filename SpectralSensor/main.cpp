#include "AS7341.h"
#include "SpectralSensor.h"
#include <stdexcept>

//#include "SpectrumData.h"
#include "ColorRenderingIndex.h"
#include "TestData.h"

int main()
{
    printf("*** Spectral sensor v0.1 ***\n\n");

#ifdef VERIFY_CALCS

    printf("*** VERIFICATION MODE ***\n\n");

    SpectralSensor sensor;
    sensor.loadCorrectionMatrix();

    // sensor.checkChannelDataCalcs();
    // sensor.checkCIE1931Calcs(10);
    // sensor.checkCIE1931Calcs(CHANNEL_F8);
    //sensor.verifySpectralReconstruction();

    try
    {
        printf("\n*** verify CRI calculation ***\n");

        // TCS table
        uint8_t Ri[MAX_TCS];

        ColorRenderingIndex cri;
        cri.loadTCSTable();
        cri.calculateCRI2(/*FW3A_2700*/ white_2700_osram, Ri);

        printf("Result:\n");
        for (uint8_t i = 0; i < MAX_TCS; i++)
        {
            printf("R%d: %d | ", i + 1, Ri[i]);
        }
        printf("\n");

        // Ra based on R1 to R8
        // uint16_t Rsum = accumulate(vector.begin(), vector.end(), ecltype(vector)::value_type(0));
        // uint8_t Ra = Rsum / 8;
    }
    catch (const std::exception &e)
    {
        printf("Exception: %s\n", e.what());
    }

#else

    try
    {
        SpectralSensor sensor;

        // setup
        sensor.setupI2C();
        sensor.setupAS7341();
        sensor.loadCorrectionMatrix();

        // take n readings
        /*for (uint8_t readings = 0; readings < 5; readings++)
        {*/
            sensor.takeReading();
        //}
    }
    catch (const std::exception &e)
    {
        printf("Exception: %s\n", e.what());
    }

#endif
}
