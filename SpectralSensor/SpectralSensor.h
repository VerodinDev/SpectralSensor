#pragma once

#include <cstdint>

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
    void takeReading();

#ifdef VERIFY_CALCS
    void checkChannelDataCalcs();
    void checkCIE1931Calcs(uint8_t lastChannel);
    void verifySpectralReconstruction();
#endif

  private:

    // I2C controller
    MCP2221 *m_pI2cController;

    // spectral sensor
    AS7341 *m_pSensor;
};
