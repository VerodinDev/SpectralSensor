#include "SpectralSensor.h"
#include <stdexcept>
#include "AS7341.h"

int main()
{
    printf("*** Spectral sensor v0.1 ***\n");

#ifdef VERIFY_CALCS

    SpectralSensor sensor;

    //sensor.checkChannelDataCalcs();
    sensor.checkCIE1931Calcs(10);
    //sensor.checkCIE1931Calcs(CHANNEL_F8);

    //sensor.verifySpectralReconstruction();

#else

    try
    {
        SpectralSensor sensor;

        // setup
        sensor.setupI2C();
        sensor.setupAS7341();

        // take n readings
        for (uint8_t readings = 0; readings < 10; readings++)
        {
            sensor.takeReading();
        }
    }
    catch (const std::exception &e)
    {
        printf("Exception: %s\n", e.what());
    }

#endif

}
