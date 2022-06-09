#pragma once

#include "AS7341_I2C_registers.h"
#include "MCP2221.h"
#include <stdint.h>

// TODO
struct ChannelValues
{
    as7341_color_channel_t channel;
    uint16_t raw;
    double basicCount;
    double correctedCount;
    double normalisedCount;
};

class AS7341
{
  public:
    AS7341(MCP2221 &i2cController);
    ~AS7341();

    bool init();

    bool setASTEP(uint16_t astep_value);
    bool setATIME(uint8_t atime_value);
    bool setGain(as7341_gain_t gain_value);

    uint16_t getASTEP();
    uint8_t getATIME();
    as7341_gain_t getGain();

    double getTINT();

    bool readAllChannels(void);
    bool readAllChannels(uint16_t *readings_buffer);
    void delayForData(uint16_t waitTime = 0);

    void setup_F1F4_Clear_NIR(void);
    void setup_F5F8_Clear_NIR(void);

    void powerEnable(bool enable_power);
    bool enableSpectralMeasurement(bool enable_measurement);

    bool enableLED(bool enable_led);
    bool setLEDCurrent(uint16_t led_current_ma);

    bool getIsDataReady();
    bool setBank(bool low); // low true gives access to 0x60 to 0x74

    // calculations
    double toBasicCounts(uint16_t raw);
    void calculateBasicCounts();
    void applyGainCorrection(double corrections[]);
    void calculateDataSensorCorrection();

    // getters
    uint16_t getChannel(as7341_color_channel_t channel) const;
    double getBasicCount(as7341_color_channel_t channel) const;
    double getCorrectedCount(as7341_color_channel_t channel) const;
    void getCorrectedCounts(double[]) const;

  private:
    // AS7341();

    bool enableSMUX(void);
    bool setSMUXCommand(as7341_smux_cmd_t command);
    void setSMUXLowChannels(bool f1_f4);
    void normalise();
    void enableAutoGain();
    uint8_t getAStatus();

    MCP2221 m_i2c;
    as7341_waiting_t m_readingState;

    // The value of the last reading of the spectral interrupt source register
    uint8_t m_last_spectral_int_source;

    uint16_t m_channel_readings[12];
    double m_basicCounts[10];
    double m_correctedCounts[10];
    double m_normalisedValues[10];

    uint8_t getOptimizedMeasurementValues(bool checkState, bool optimizedValuesDetected, uint16_t rawVal[],
                                          double basicVal[], double corrVal[]);
};
