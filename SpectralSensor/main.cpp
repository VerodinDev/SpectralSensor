#include "SpectralSensor.h"
#include <stdexcept>

int main()
{
    printf("*** Spectral sensor v0.1 ***\n");

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
}
