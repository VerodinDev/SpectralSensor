#pragma once

#include "AS7341_I2C_registers.h"
#include "MCP2221.h"
#include <stdint.h>

enum SpectralChannel
{
    CHANNEL_F1,
    CHANNEL_F2,
    CHANNEL_F3,
    CHANNEL_F4,
    CHANNEL_F5,
    CHANNEL_F6,
    CHANNEL_F7,
    CHANNEL_F8,
    CHANNEL_CLEAR,
    CHANNEL_NIR
};

// TODO
struct ChannelValues
{
    as7341_color_channel channel;
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

    void init();

    void setASTEP(uint16_t astep_value);
    void setATIME(uint8_t atime_value);
    void setGain(as7341_gain gain_value);

    uint16_t getASTEP();
    uint8_t getATIME();
    as7341_gain getGain();

    double getTINT();

    void readAllChannels();
    void readAllChannels(uint16_t *readings_buffer);
    void delayForData(uint16_t waitTime = 0);

    void setup_F1F4_Clear_NIR();
    void setup_F5F8_Clear_NIR();

    void powerEnable(bool enable);
    void enableAutoGain(bool enable);
    void enableSpectralMeasurement(bool enable);

    void enableLED(bool enable);
    void setLEDCurrent(uint16_t current);

    bool isDataReady();
    void setBank(bool low); // low true gives access to 0x60 to 0x74

    // calculations
    double toBasicCounts(uint16_t raw);
    void calculateBasicCounts();
    void applyGainCorrection(double corrections[]);
    void calculateDataSensorCorrection();

    // getters
    uint16_t getChannel(as7341_color_channel channel) const;
    uint16_t getRawValue(SpectralChannel channel) const;
    double getBasicCount(SpectralChannel channel) const;
    double getCorrectedCount(SpectralChannel channel) const;
    void getCorrectedCounts(double[]) const;

  private:
    // AS7341();

    void enableSMUX();
    void setSMUXCommand(as7341_smux_cmd command);
    void setSMUXLowChannels(bool f1_f4);
    void normalise();
    uint8_t getAStatus();

    MCP2221 m_i2c;
    as7341_waiting m_readingState;
    as7341_gain m_gainStatus;
    bool m_isSaturated;
    bool m_useAutoGain;

    // The value of the last reading of the spectral interrupt source register
    uint8_t m_last_spectral_int_source;

    uint16_t m_channel_readings[12];
    uint16_t m_rawValues[10];
    double m_basicCounts[10];
    double m_correctedCounts[10];
    double m_normalisedValues[10];

    uint8_t getOptimizedMeasurementValues(bool checkState, bool optimizedValuesDetected, uint16_t rawVal[],
                                          double basicVal[], double corrVal[]);
};
