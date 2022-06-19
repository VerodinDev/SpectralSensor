
#include "AS7341.h"
#include "AS7341_values.h"
#include <algorithm>
#include <array>
#include <mcp2221_dll_um.h>
#include <stdexcept>
#include <windows.h> // Sleep()

using namespace std;

AS7341::AS7341(MCP2221 &i2cController)
    : m_i2c(i2cController), m_readingState(AS7341_WAITING_DONE), m_gainStatus(AS7341_GAIN_1X), m_isSaturated(false),
      m_useAutoGain(false), m_last_spectral_int_source(0), m_channel_readings{0}
{
    for (uint8_t channel = 0; channel < 10; channel++)
    {
        m_readings[channel].channel = static_cast<SpectralChannel>(channel);
        m_readings[channel].raw = 0;
        m_readings[channel].basicCount = 0;
        m_readings[channel].correctedCount = 0;
        m_readings[channel].normalisedCount = 0;
    }
}

AS7341::~AS7341()
{
}

void AS7341::init()
{
    uint8_t ids[3];
    m_i2c.readRegister(AS7341_AUXID, ids, 3);

    printf("Device ID = 0x%X, rev ID = 0x%02X, Aux ID = 0x%02X\n", ids[2], ids[1] & 0xF, ids[0] & 0x7);

    // make sure we're talking to the right chip
    if ((ids[2] & 0xFC) != (AS7341_CHIP_ID << 2))
    {
        throw std::runtime_error("Device ID mismatch. Wrong chip?");
    }

    powerEnable(true);
}

// read spectral channels
void AS7341::readAllChannels(uint16_t *readings_buffer)
{
    printf("Reading all channels\n");

    // turn on auto gain
    // do this before taking actual readings or the result is incorrect (affects TINT?)
    enableAutoGain(true);

    // read high channels (need F8 for auto gain)
    // just use same buffer, will be overwritten anyway
    setSMUXLowChannels(false);
    enableSpectralMeasurement(true);
    delayForData(0);

    m_i2c.readRegister(AS7341_CH0_DATA_L, (uint8_t *)&readings_buffer[6], 12);

    getAStatus();

    if (m_isSaturated)
    {
        printf("Saturated\n");
    }

    enableAutoGain(false);

    // now take actual readings
    enableSpectralMeasurement(true);
    delayForData(0);

    m_i2c.readRegister(AS7341_CH0_DATA_L, (uint8_t *)&readings_buffer[6], 12);

    // read low channels
    setSMUXLowChannels(true);
    enableSpectralMeasurement(true);
    delayForData(0);

    m_i2c.readRegister(AS7341_CH0_DATA_L, (uint8_t *)readings_buffer, 12);

#ifdef VERIFY_CALCS_AS7341

    for (uint8_t channel = 0; channel < 10; channel++)
    {
        m_readings[channel].raw = tstRawCounts[channel];
    }

#else

    // strip out CLEAR and NIR channels in the middle
    m_readings[CHANNEL_F1].raw = m_channel_readings[AS7341_CHANNEL_F1];
    m_readings[CHANNEL_F2].raw = m_channel_readings[AS7341_CHANNEL_F2];
    m_readings[CHANNEL_F3].raw = m_channel_readings[AS7341_CHANNEL_F3];
    m_readings[CHANNEL_F4].raw = m_channel_readings[AS7341_CHANNEL_F4];
    m_readings[CHANNEL_F5].raw = m_channel_readings[AS7341_CHANNEL_F5];
    m_readings[CHANNEL_F6].raw = m_channel_readings[AS7341_CHANNEL_F6];
    m_readings[CHANNEL_F7].raw = m_channel_readings[AS7341_CHANNEL_F7];
    m_readings[CHANNEL_F8].raw = m_channel_readings[AS7341_CHANNEL_F8];
    m_readings[CHANNEL_CLEAR].raw = m_channel_readings[AS7341_CHANNEL_CLEAR];
    m_readings[CHANNEL_NIR].raw = m_channel_readings[AS7341_CHANNEL_NIR];

#endif
}

// delay while waiting for data, with option to time out and recover
void AS7341::delayForData(uint16_t waitTime)
{
    if (waitTime == 0)
    {
        // wait forever
        while (!isDataReady())
        {
            Sleep(1);
        }
        return;
    }

    if (waitTime > 0)
    {
        uint32_t elapsedMillis = 0;

        while (!isDataReady() && elapsedMillis < waitTime)
        {
            Sleep(1);
            elapsedMillis++;
        }
        return;
    }
}

// read channels F1-8, Clear and NIR
// TODO pointless, remove
void AS7341::readAllChannels()
{
    readAllChannels(m_channel_readings);
}

void AS7341::setSMUXLowChannels(bool f1_f4)
{
    enableSpectralMeasurement(false);
    setSMUXCommand(AS7341_SMUX_CMD_WRITE);

    if (f1_f4)
    {
        setup_F1F4_Clear_NIR();
    }
    else
    {
        setup_F5F8_Clear_NIR();
    }

    enableSMUX();
}

// set power state of sensor
void AS7341::powerEnable(bool enable_power)
{
    uint8_t enable_reg = m_i2c.readRegisterByte(AS7341_ENABLE);

    // set PON bit
    enable_reg = m_i2c.modifyBitInByte(enable_reg, (uint8_t)enable_power, 0);
    m_i2c.writeRegisterByte(AS7341_ENABLE, enable_reg);
}

// enable measurement of spectral data
void AS7341::enableSpectralMeasurement(bool enable)
{
    uint8_t enable_reg = m_i2c.readRegisterByte(AS7341_ENABLE);
    enable_reg = m_i2c.modifyBitInByte(enable_reg, (uint8_t)enable, 1);

    m_i2c.writeRegisterByte(AS7341_ENABLE, enable_reg);
}

void AS7341::enableSMUX()
{
    m_i2c.modifyRegisterBit(AS7341_ENABLE, true, 4);

    int timeOut = 1000; // arbitrary value, but if it takes 1000 milliseconds then something is wrong
    int count = 0;

    while (m_i2c.checkRegisterBit(AS7341_ENABLE, 4) && count < timeOut)
    {
        Sleep(1);
        count++;
    }

    if (count >= timeOut)
    {
        throw runtime_error("Enable SMUX timeout");
    }
}

void AS7341::setSMUXCommand(as7341_smux_cmd command)
{
    m_i2c.modifyRegisterMultipleBit(AS7341_CFG6, command, 3, 2);
}

// enable LED
void AS7341::enableLED(bool enable)
{
    printf("Set LED to %d\n", enable);

    // access 0x60-0x74
    setBank(true);

    m_i2c.modifyRegisterBit(AS7341_CONFIG, enable, 3);
    m_i2c.modifyRegisterBit(AS7341_LED, enable, 7);

    // access registers 0x80 and above (default)
    setBank(false);
}

// set current limit for the LED
void AS7341::setLEDCurrent(uint16_t current)
{
    // check within permissible range
    if (current > 258)
    {
        throw range_error("LED current too high");
    }
    if (current < 4)
    {
        current = 4;
    }

    // Access 0x60 0x74
    setBank(true);

    m_i2c.modifyRegisterMultipleBit(AS7341_LED, (uint8_t)((current - 4) / 2), 0, 7);

    // Access registers 0x80 and above (default)
    setBank(false);
}

// set active register bank
void AS7341::setBank(bool low)
{
    m_i2c.modifyRegisterBit(AS7341_CFG0, low, 4);
}

bool AS7341::isDataReady()
{
    return m_i2c.checkRegisterBit(AS7341_STATUS2, 6);
}

// configure SMUX for sensors F1-4, Clear and NIR
void AS7341::setup_F1F4_Clear_NIR()
{
    // SMUX Config for F1,F2,F3,F4,NIR,Clear
    m_i2c.writeRegisterByte(0x00, 0x30); // F3 left set to ADC2
    m_i2c.writeRegisterByte(0x01, 0x01); // F1 left set to ADC0
    m_i2c.writeRegisterByte(0x02, 0x00); // Reserved or disabled
    m_i2c.writeRegisterByte(0x03, 0x00); // F8 left disabled
    m_i2c.writeRegisterByte(0x04, 0x00); // F6 left disabled
    m_i2c.writeRegisterByte(0x05, 0x42); // F4 left connected to ADC3/f2 left connected to ADC1
    m_i2c.writeRegisterByte(0x06, 0x00); // F5 left disbled
    m_i2c.writeRegisterByte(0x07, 0x00); // F7 left disbled
    m_i2c.writeRegisterByte(0x08, 0x50); // CLEAR connected to ADC4
    m_i2c.writeRegisterByte(0x09, 0x00); // F5 right disabled
    m_i2c.writeRegisterByte(0x0A, 0x00); // F7 right disabled
    m_i2c.writeRegisterByte(0x0B, 0x00); // Reserved or disabled
    m_i2c.writeRegisterByte(0x0C, 0x20); // F2 right connected to ADC1
    m_i2c.writeRegisterByte(0x0D, 0x04); // F4 right connected to ADC3
    m_i2c.writeRegisterByte(0x0E, 0x00); // F6/F8 right disabled
    m_i2c.writeRegisterByte(0x0F, 0x30); // F3 right connected to AD2
    m_i2c.writeRegisterByte(0x10, 0x01); // F1 right connected to AD0
    m_i2c.writeRegisterByte(0x11, 0x50); // CLEAR right connected to AD4
    m_i2c.writeRegisterByte(0x12, 0x00); // Reserved or disabled
    m_i2c.writeRegisterByte(0x13, 0x06); // NIR connected to ADC5
}

// configure SMUX for sensors F5-8, Clear and NIR
void AS7341::setup_F5F8_Clear_NIR()
{
    // SMUX Config for F5,F6,F7,F8,NIR,Clear
    m_i2c.writeRegisterByte(0x00, 0x00); // F3 left disable
    m_i2c.writeRegisterByte(0x01, 0x00); // F1 left disable
    m_i2c.writeRegisterByte(0x02, 0x00); // reserved/disable
    m_i2c.writeRegisterByte(0x03, 0x40); // F8 left connected to ADC3
    m_i2c.writeRegisterByte(0x04, 0x02); // F6 left connected to ADC1
    m_i2c.writeRegisterByte(0x05, 0x00); // F4/ F2 disabled
    m_i2c.writeRegisterByte(0x06, 0x10); // F5 left connected to ADC0
    m_i2c.writeRegisterByte(0x07, 0x03); // F7 left connected to ADC2
    m_i2c.writeRegisterByte(0x08, 0x50); // CLEAR Connected to ADC4
    m_i2c.writeRegisterByte(0x09, 0x10); // F5 right connected to ADC0
    m_i2c.writeRegisterByte(0x0A, 0x03); // F7 right connected to ADC2
    m_i2c.writeRegisterByte(0x0B, 0x00); // Reserved or disabled
    m_i2c.writeRegisterByte(0x0C, 0x00); // F2 right disabled
    m_i2c.writeRegisterByte(0x0D, 0x00); // F4 right disabled
    m_i2c.writeRegisterByte(0x0E, 0x24); // F8 right connected to ADC2/ F6 right connected to ADC1
    m_i2c.writeRegisterByte(0x0F, 0x00); // F3 right disabled
    m_i2c.writeRegisterByte(0x10, 0x00); // F1 right disabled
    m_i2c.writeRegisterByte(0x11, 0x50); // CLEAR right connected to AD4
    m_i2c.writeRegisterByte(0x12, 0x00); // Reserved or disabled
    m_i2c.writeRegisterByte(0x13, 0x06); // NIR connected to ADC5
}

// set ATIME
void AS7341::setATIME(uint8_t atime_value)
{
    m_i2c.writeRegisterByte(AS7341_ATIME, atime_value);
}

// get ATIME
uint8_t AS7341::getATIME()
{
    return m_i2c.readRegisterByte(AS7341_ATIME);

    // debug
    // uint8_t atime = m_i2c.readRegisterByte(AS7341_ATIME);
    // printf("ATIME = %d\n", atime);
    // return atime;
}

// set ASTEP
void AS7341::setASTEP(uint16_t astep_value)
{
    // for some reason no matter what I do, ASTEP ends up wrong in the registers. Doing 2 seperate writes works fine.
    //m_i2c.writeRegister(AS7341_ASTEP_L, (uint8_t *)&astep_value, 2);

    m_i2c.writeRegisterByte(AS7341_ASTEP_L, astep_value & 0x00FF);
    m_i2c.writeRegisterByte(AS7341_ASTEP_H, (astep_value >> 8) & 0xFF);
}

// get ASTEP
uint16_t AS7341::getASTEP()
{
    uint8_t data[2] = {0};

    m_i2c.readRegister(AS7341_ASTEP_L, data, 2);
    return (((uint16_t)data[1]) << 8) | data[0];

    // debug
    // uint16_t astep = (((uint16_t)data[1]) << 8) | data[0];
    // printf("ASTEP = %d\n", astep);
    // return astep;
}

// set ADC gain multiplier
void AS7341::setGain(as7341_gain gain_value)
{
    m_i2c.writeRegisterByte(AS7341_CFG1, gain_value);

    // AGAIN bitfield is only[0:4] but the rest is empty
}

// get ADC gain multiplier
as7341_gain AS7341::getGain()
{
#ifdef VERIFY_CALCS_AS7341

    return AS7341_GAIN_4X;

#else

    return m_useAutoGain ? m_gainStatus : static_cast<as7341_gain>(m_i2c.readRegisterByte(AS7341_CFG1));

#endif

    // return static_cast<as7341_gain>(m_i2c.readRegisterByte(AS7341_CFG1));

    // debug
    // as7341_gain_t gain = static_cast<as7341_gain>(m_i2c.readRegisterByte(AS7341_CFG1));
    // printf("Gain = %d\n", gain);
    // return gain;
}

// calculate integration time
double AS7341::getTINT()
{
    uint16_t astep = getASTEP();
    uint8_t atime = getATIME();

    // SPM/SYNS mode: Tint = (ATIME + 1) * (ASTEP + 1) * 2.78μS
    // SYND mode: Tint = ITIME × 2,78μS
    // default values 999 and 0 -> 50 ms

    // 30 * 600 * 2.78 = 50040
    // TODO fix warnings
    // C26451	Arithmetic overflow : Using operator '*' on a 4 byte value and then casting the result to a 8 byte value.Cast the value to the wider type before calling operator '*' to avoid overflow(io.2).
    double tint = (atime + 1) * (astep + 1) * 2.78 / 1000;
    // printf("TINT = %f (atime= %d, astep=%d)\n", tint, atime, astep);
    return tint;
}

// convert raw values to basic counts
// basic counts = raw counts / (gain * integration time)
double AS7341::toBasicCounts(uint16_t raw)
{
    float gain_val = 0;
    as7341_gain gain = getGain();

    switch (gain)
    {
    case AS7341_GAIN_0_5X:
        gain_val = 0.5;
        break;

    case AS7341_GAIN_1X:
        gain_val = 1;
        break;

    case AS7341_GAIN_2X:
        gain_val = 2;
        break;

    case AS7341_GAIN_4X:
        gain_val = 4;
        break;

    case AS7341_GAIN_8X:
        gain_val = 8;
        break;

    case AS7341_GAIN_16X:
        gain_val = 16;
        break;

    case AS7341_GAIN_32X:
        gain_val = 32;
        break;

    case AS7341_GAIN_64X:
        gain_val = 64;
        break;

    case AS7341_GAIN_128X:
        gain_val = 128;
        break;

    case AS7341_GAIN_256X:
        gain_val = 256;
        break;

    case AS7341_GAIN_512X:
        gain_val = 512;
        break;
    }

    return raw / (gain_val * getTINT());
}

// get specific color channel
uint16_t AS7341::getChannel(as7341_color_channel channel) const
{
    return m_channel_readings[channel];
}

uint16_t AS7341::getRawValue(SpectralChannel channel) const
{
    return m_readings[channel].raw;
}

// The higher the counts (before saturation), the better the accuracy. Changing gain or TINT will affect
// counts.Both parameters will have different effects like FSR, noise, linearities, time, and others.
double AS7341::getBasicCount(SpectralChannel channel) const
{
    return m_readings[channel].basicCount;
}

double AS7341::getCorrectedCount(SpectralChannel channel) const
{
    return m_readings[channel].correctedCount;
}

void AS7341::getCorrectedCounts(double counts[]) const
{
    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
    {
        counts[i] = m_readings[i].correctedCount;
    }

    // std::array<double, 10> counts = m_correctedCounts;
    // return &m_correctedCounts[0];
}

void AS7341::calculateBasicCounts()
{
    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        m_readings[channel].basicCount = toBasicCounts(m_readings[channel].raw);
    }
}

void AS7341::applyGainCorrection(double corrections[])
{
    as7341_gain gain = getGain();

    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        m_readings[channel].basicCount *= corrections[gain];
        // printf("channel %d\n", channel);
    }
}

// calculate data sensor corr (corrected counts)
// value = correctionFactor * (basic counts - offsetCompensationValue)
void AS7341::calculateDataSensorCorrection()
{
    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        m_readings[channel].correctedCount = correctionFactors_v3[channel] * (m_readings[channel].basicCount - offsetCompensationValues_v3[channel]);
    }

    normalise();
}

void AS7341::normalise()
{
    double highestValue(0);

    // std::max not compiling
    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
    {
        if (m_readings[i].correctedCount > highestValue)
        {
            highestValue = m_readings[i].correctedCount;
        }
    }

    // printf("max = %f\n", highestValue);

    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
    {
        m_readings[i].correctedCount /= highestValue;
    }
}

void AS7341::enableAutoGain(bool enable)
{
    // enable spectral AGC (SP_AGC)
    uint8_t cfg8_reg = m_i2c.readRegisterByte(AS7341_CFG8);
    cfg8_reg = m_i2c.modifyBitInByte(cfg8_reg, (uint8_t)enable, 2);
    m_i2c.writeRegisterByte(AS7341_CFG8, cfg8_reg);

    m_useAutoGain = enable;

    if (!enable)
    {
        return;
    }

    // set spectral threshold channel (SP_TH_CH) to use clear channel
    uint8_t thresholdChannel = AS7341_ADC_CHANNEL_4;

    uint8_t cfg12_reg = m_i2c.readRegisterByte(AS7341_CFG12);
    m_i2c.modifyRegisterMultipleBit(cfg12_reg, thresholdChannel, 0, 2);
    m_i2c.writeRegisterByte(AS7341_CFG12, cfg12_reg);

    // Sets the channel used for interrupts, persistence and
    //	the AGC, if enabled, to determine device statusand
    //	gain settings.

    // set thresholds (AGC_H and AGC_L)
    uint8_t lowHysteresis = AGC_L_50;
    uint8_t highHysteresis = AGC_H_87;

    // The threshold is automatically calculated internally as a percentage of full - scale.Note that full - scale is
    // equal to(ATIME + 1) x(ASTEP + 1).
    uint8_t cfg10_reg = m_i2c.readRegisterByte(AS7341_CFG10);
    m_i2c.modifyRegisterMultipleBit(cfg10_reg, lowHysteresis, 4, 2);
    m_i2c.modifyRegisterMultipleBit(cfg10_reg, highHysteresis, 6, 2);
    m_i2c.writeRegisterByte(AS7341_CFG10, cfg10_reg);

    // AS7341_AGC_GAIN_MAX
    //	3 : 0 AGC_AGAIN_MAX
    //	AGC Gain Max.
    //	Sets the maximum gain for AGC engine to
    //	2......_...._........_......
    //	Default value is 9 (256x).The range can be set from
    //	0 (0.5x) to 10 (512x).

    // FIFO_WRITE_ASTATUS???
}

uint8_t AS7341::getAStatus()
{
    uint8_t astatus_reg = m_i2c.readRegisterByte(AS7341_ASTATUS);

    // 3:0 AGAIN_STATUS
    // required to calculate spectral results if AGC is enabled
    m_gainStatus = static_cast<as7341_gain>(astatus_reg & 0x07);

    // 7 ASAT_STATUS
    // Indicates if the latched data is affected by analog or digital saturation
    m_isSaturated = astatus_reg & 0x80;

    printf("saturation = %d, gain status = %d\n", m_isSaturated, m_gainStatus);

    return true;
}
