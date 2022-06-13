
#include "AS7341.h"
#include <algorithm>
#include <array>
#include <mcp2221_dll_um.h>
#include <stdexcept>
#include <windows.h> // Sleep()

using namespace std;

AS7341::AS7341(MCP2221 &i2cController)
    : m_i2c(i2cController), m_readingState(AS7341_WAITING_DONE), m_gainStatus(AS7341_GAIN_1X), m_isSaturated(false),
      m_useAutoGain(false),
      m_last_spectral_int_source(0), m_basicCounts{0}, m_channel_readings{0}, m_correctedCounts{0}, m_normalisedValues{
                                                                                                        0}
{
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

    // strip out CLEAR and NIR channels in the middle
    m_rawValues[CHANNEL_F1] = m_channel_readings[AS7341_CHANNEL_F1];
    m_rawValues[CHANNEL_F2] = m_channel_readings[AS7341_CHANNEL_F2];
    m_rawValues[CHANNEL_F3] = m_channel_readings[AS7341_CHANNEL_F3];
    m_rawValues[CHANNEL_F4] = m_channel_readings[AS7341_CHANNEL_F4];
    m_rawValues[CHANNEL_F5] = m_channel_readings[AS7341_CHANNEL_F5];
    m_rawValues[CHANNEL_F6] = m_channel_readings[AS7341_CHANNEL_F6];
    m_rawValues[CHANNEL_F7] = m_channel_readings[AS7341_CHANNEL_F7];
    m_rawValues[CHANNEL_F8] = m_channel_readings[AS7341_CHANNEL_F8];
    m_rawValues[CHANNEL_CLEAR] = m_channel_readings[AS7341_CHANNEL_CLEAR];
    m_rawValues[CHANNEL_NIR] = m_channel_readings[AS7341_CHANNEL_NIR];
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
    // return m_i2c.writeRegister(AS7341_ASTEP_L, (uint8_t *)&astep_value, 2);

    m_i2c.writeRegisterByte(AS7341_ASTEP_L, 0x57);
    m_i2c.writeRegisterByte(AS7341_ASTEP_H, 0x02);
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
    return m_useAutoGain ? m_gainStatus : static_cast<as7341_gain>(m_i2c.readRegisterByte(AS7341_CFG1));

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
    return m_rawValues[channel];
}

// The higher the counts (before saturation), the better the accuracy. Changing gain or TINT will affect
// counts.Both parameters will have different effects like FSR, noise, linearities, time, and others.
double AS7341::getBasicCount(SpectralChannel channel) const
{
    return m_basicCounts[channel];
}

double AS7341::getCorrectedCount(SpectralChannel channel) const
{
    return m_correctedCounts[channel];
}

void AS7341::getCorrectedCounts(double counts[]) const
{
    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
        counts[i] = m_correctedCounts[i];
    // std::array<double, 10> counts = m_correctedCounts;
    // return &m_correctedCounts[0];
}

void AS7341::calculateBasicCounts()
{
    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        m_basicCounts[channel] = toBasicCounts(m_rawValues[channel]);
    }
}

void AS7341::applyGainCorrection(double corrections[])
{
    as7341_gain gain = getGain();

    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        m_basicCounts[channel] *= corrections[gain];
        printf("channel %d\n", channel);
    }
}

// calculate data sensor corr (corrected counts)
// value = correctionFactor * (basic counts - offsetCompensationValue)
void AS7341::calculateDataSensorCorrection()
{
    for (uint8_t channel = CHANNEL_F1; channel <= CHANNEL_NIR; channel++)
    {
        // TODO
        // m_correctedCounts[channel] = correctionFactors[channel] * (m_basicCounts[channel] -
        // offsetCompensationValues[channel]);
        // ALS Corrected_Counts = Basic_Counts ∗ Gain_Correction ∗ Correction_Factor - Offset
        m_correctedCounts[channel] = m_basicCounts[channel];
    }

    normalise();
}

void AS7341::normalise()
{
    double highestValue(0);

    // std::max not compiling
    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
    {
        if (m_correctedCounts[i] > highestValue)
            highestValue = m_correctedCounts[i];
    }

    printf("max = %f\n", highestValue);

    for (uint8_t i = CHANNEL_F1; i <= CHANNEL_NIR; i++)
        m_correctedCounts[i] /= highestValue;
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

// AN000651 - AS7341 Auto Gain & Optimization
// Auto Gain& optimization will automatically find the best(maximum possible) parameter options for the
// gainand get the maximum optimized raw value in a defined range.Optimization analyzes sensor output
// and the gain parameter setup to find an optimal ADC value based on various calculations of the gain.
//
// The steps in Auto Gain optimization is divided into two sections.One part contains the Auto Gain, and
// the second part optimizes the derived Auto Gain.In the Auto Gain section, a gain between the maximum
// and minimum range is automatically calculated by the results of the test measurement.Therefore, the
// sensor’s raw value is placed as close to the maximum as possible, without saturation.
uint8_t AS7341::getOptimizedMeasurementValues(bool checkState, bool optimizedValuesDetected, uint16_t rawVal[],
                                              double basicVal[], double corrVal[])
{
    //	uint8_t currentGain, maxGain;
    //	int errorcode;
    //	int measureCount;
    //
    //	RawValueStates rawValueState = RawValueStates.Saturation;
    //
    //	// start with middle gain value
    //	// middle Gain value is calculated from the maximum possible gain for optimizationand taken as the currentGain.
    //	currentGain = (uint8_t)(cobMaxGain.Items.Count / 2.0 + 0.5);
    //
    //	if (currentGain > maxGain)
    //	{
    //		currentGain = maxGain;
    //	}
    //
    //	uint8_t newGain;
    //	uint8_t saturationGain = (uint8_t)(cobMaxGain.Items.Count + 1);
    //
    //	// Inside the while loop, the gain of the device is set to currentGain.Then, reads out the raw
    //	// measurementand checks the saturation or noise state of rawValue measurements.If any of the
    //	// conditions are true, it will enter the corresponding loop.
    //	// If the raw value is above the maximum range of the raw values(RawValueStates.Saturation) gain
    //	// correction is made by reducing the gain by half of the currentGain using the algorithm below.
    //	while (true)
    //	{
    //		// set gain value
    //		errorcode = setGain(currentGain);
    //		if (errorcode != OK)
    //		{
    //			//throw new Exception("Error from setGain: " + ((As7341Errorcodes)errorcode).ToString());
    //		}
    //
    //		// measure and check raw vlues
    //		rawValueState = CheckRawValues(ref checkState, ref rawVal, ref basicVal, ref corrVal);
    //		measureCount++;
    //
    //		if (rawValueState == RawValueStates.Saturation)
    //		{
    //			//BaseFunctions.DebugOut(true, "Saturation gain: " + currentGain.ToString());
    //			// in case of saturation have the gain value
    //			if (currentGain == 0)
    //			{
    //				break;
    //			}
    //
    //			// current gain less than saturation gain, then setting saturationGain equals currentGain
    //			if (currentGain < saturationGain)
    //			{
    //				saturationGain = currentGain
    //			}
    //
    //			// set new gain value
    //			currentGain >>= 1;
    //		}
    //
    //		//Otherwise, if the raw value is below the minimum range of raw values(RawValueStates.Noise),
    //		//	the gain correction is made by increasing the gain when it is in noise state is shown in the
    //		//	algorithm below.
    //		else if (rawValueState == RawValueStates.Noise)
    //		{
    //			//BaseFunctions.DebugOut(true, "Noise gain: " + currentGain.ToString() + " Raw: " +
    // rawVal.Max().ToString());
    //			// in case of low gain value use the middle between max and current gain
    //			if (currentGain == maxGain)
    //			{
    //				break;
    //			}
    //
    //			newGain = (byte)((maxGain + currentGain) / 2.0 + 0.5);
    //
    //			if (newGain == currentGain)
    //			{
    //				newGain++;
    //			}
    //
    //			// check if new gain value is greater than saturationGain flag
    //			if (newGain >= saturationGain)
    //			{
    //				break;
    //			}
    //			currentGain = newGain;
    //		}
    //		else
    //		{
    //			//BaseFunctions.DebugOut(true, "Ok gain: " + currentGain.ToString() + " Raw: " +
    // rawVal.Max().ToString()); 			break;
    //		}
    //	}
    //
    //	// check for saturation
    //	if (rawValueState == RawValueStates.Saturation)
    //	{
    //		//lblOptimizationError.ForeColor = Color.Red;
    //		//lblOptimizationError.Text = "Optimization not possible due to saturation";
    //		return errorcode;
    //	}
    //
    //	// set values for optimization
    //	uint16_t currentRawVal = rawVal.Max();
    //	double maxRawVal = _sensor.MaxCounts * _maximumAdcRange;
    //	double minRawVal = _sensor.MaxCounts * _minimumAdcRange;
    //
    //	// optimize gain
    //	// Log(Double, Double) Returns the logarithm of a specified number in a specified base.
    //	int diffGain = (int)floor(log(maxRawVal / currentRawVal, 2));
    //	if (currentGain + diffGain > maxGain)
    //	{
    //		diffGain = maxGain - currentGain;
    //	}
    //
    //	currentRawVal = (uint16_t)(diffGain >= 0 ? currentRawVal << diffGain : currentRawVal >> -diffGain);
    //	currentGain = (uint8_t)((int)currentGain + diffGain);
    //
    //	// set gain value
    //	errorcode = setGain(currentGain);
    //	if (errorcode != OK)
    //	{
    //		//throw new Exception("Error from setGain: " + As7341Errorcodes)errorcode).ToString());
    //	}
    //	// show current values
    //	cobAgain.SelectedIndex = currentGain;
    //	lblResultAgain.Text = _sensor.calculateGain(currentGain) + "x";
    //
    //	// measurement with optimized values
    //	errorcode = getMeasurementValues(checkState, rawVal, basicVal, corrVal);
    //	//lblOptimizationMessage.ForeColor = Color.Black;
    //	//lblOptimizationMessage.Text = "Optimization measurements: " + ++measureCount;
    //	return errorcode;

    return 0;
}
