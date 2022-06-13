#pragma once

class MCP2221;
class AS7341;

class SpectralSensor
{
  public:
    SpectralSensor();
    // virtual ~SpectralSensor();

    void setupI2C();
    void setupAS7341();
    void takeReading();

  private:
    // I2C controller
    MCP2221 *m_pI2cController;

    // spectral sensor
    AS7341 *m_pSensor;
};
