#pragma once

#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI 0x92 ///< Chip ID register

#define AS7341_ASTATUS_SYND 0x60    ///< AS7341_ASTATUS, SYND mode
#define AS7341_CH0_DATA_L_SYND 0x61 ///< AS7341_CH0_DATA_L, SYND mode
#define AS7341_CH0_DATA_H_SYND 0x62 ///< AS7341_CH0_DATA_H, SYND mode
#define AS7341_ITIME_L 0x63         ///< AS7341_ITIME_L
#define AS7341_ITIME_M 0x64         ///< AS7341_ITIME_M
#define AS7341_ITIME_H 0x65         ///< AS7341_ITIME_H
#define AS7341_CONFIG 0x70          ///< Enables LED control and sets light sensing mode
#define AS7341_STAT 0x71            ///< AS7341_STAT
#define AS7341_EDGE 0x72            ///< AS7341_EDGE
#define AS7341_GPIO 0x73            ///< Connects photo diode to GPIO or INT pins
#define AS7341_LED 0x74             ///< LED Register; Enables and sets current limit
#define AS7341_ENABLE 0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral Measurements and Power
#define AS7341_ATIME 0x81  ///< Sets ADC integration step count
#define AS7341_WTIME 0x83  ///< AS7341_WTIME
#define AS7341_SP_LOW_TH_L 0x84  ///< Spectral measurement Low Threshold low byte
#define AS7341_SP_LOW_TH_H 0x85  ///< Spectral measurement Low Threshold high byte
#define AS7341_SP_HIGH_TH_L 0x86 ///< Spectral measurement High Threshold low byte
#define AS7341_SP_HIGH_TH_H 0x87 ///< Spectral measurement High Threshold low byte
#define AS7341_AUXID 0x90        ///< AS7341_AUXID
#define AS7341_REVID 0x91        ///< AS7341_REVID
#define AS7341_ID 0x92           ///< AS7341_ID
#define AS7341_STATUS 0x93       ///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7341_ASTATUS 0x94      ///< AS7341_ASTATUS
#define AS7341_CH0_DATA_L 0x95   ///< ADC Channel Data
#define AS7341_CH0_DATA_H 0x96   ///< ADC Channel Data
#define AS7341_CH1_DATA_L 0x97   ///< ADC Channel Data
#define AS7341_CH1_DATA_H 0x98   ///< ADC Channel Data
#define AS7341_CH2_DATA_L 0x99   ///< ADC Channel Data
#define AS7341_CH2_DATA_H 0x9A   ///< ADC Channel Data
#define AS7341_CH3_DATA_L 0x9B   ///< ADC Channel Data
#define AS7341_CH3_DATA_H 0x9C   ///< ADC Channel Data
#define AS7341_CH4_DATA_L 0x9D   ///< ADC Channel Data
#define AS7341_CH4_DATA_H 0x9E   ///< ADC Channel Data
#define AS7341_CH5_DATA_L 0x9F   ///< ADC Channel Data
#define AS7341_CH5_DATA_H 0xA0   ///< ADC Channel Data
#define AS7341_STATUS2 0xA3      ///< Measurement status flags; saturation, validity
#define AS7341_STATUS3 0xA4      ///< Spectral interrupt source, high or low threshold
#define AS7341_STATUS5 0xA6      ///< AS7341_STATUS5
#define AS7341_STATUS6 0xA7      ///< AS7341_STATUS6
#define AS7341_CFG0 0xA9         ///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7341_CFG1 0xAA         ///< Controls ADC Gain
#define AS7341_CFG3 0xAC         ///< AS7341_CFG3
#define AS7341_CFG6 0xAF         ///< Used to configure Smux
#define AS7341_CFG8 0xB1         ///< AS7341_CFG8
#define AS7341_CFG9 0xB2         ///< Enables flicker detection and smux command completion system interrupts
#define AS7341_CFG10 0xB3        ///< AS7341_CFG10
#define AS7341_CFG12 0xB5        ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7341_PERS 0xBD         ///< Number of measurement cycles outside thresholds to trigger an interupt
#define AS7341_GPIO2 0xBE        ///< GPIO Settings and status: polarity, direction, sets output, reads input
#define AS7341_ASTEP_L 0xCA      ///< Integration step size low byte
#define AS7341_ASTEP_H 0xCB      ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX 0xCF ///< AS7341_AGC_GAIN_MAX
#define AS7341_AZ_CONFIG 0xD6    ///< AS7341_AZ_CONFIG
#define AS7341_FD_TIME1 0xD8     ///< Flicker detection integration time low byte
#define AS7341_FD_TIME2 0xDA     ///< Flicker detection gain and high nibble
#define AS7341_FD_CFG0 0xD7      ///< AS7341_FD_CFG0
#define AS7341_FD_STATUS 0xDB    ///< Flicker detection status; measurement valid, saturation, flicker type
#define AS7341_INTENAB 0xF9      ///< Enables individual interrupt types
#define AS7341_CONTROL 0xFA      ///< Auto-zero, fifo clear, clear SAI active
#define AS7341_FIFO_MAP 0xFC     ///< AS7341_FIFO_MAP
#define AS7341_FIFO_LVL 0xFD     ///< AS7341_FIFO_LVL
#define AS7341_FDATA_L 0xFE      ///< AS7341_FDATA_L
#define AS7341_FDATA_H 0xFF      ///< AS7341_FDATA_H

#define AS7341_SPECTRAL_INT_HIGH_MSK 0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7341_SPECTRAL_INT_LOW_MSK 0b00010000  ///< bitmask to check for a low threshold interrupt

// gain multipliers
enum as7341_gain
{
    AS7341_GAIN_0_5X,
    AS7341_GAIN_1X,
    AS7341_GAIN_2X,
    AS7341_GAIN_4X,
    AS7341_GAIN_8X,
    AS7341_GAIN_16X,
    AS7341_GAIN_32X,
    AS7341_GAIN_64X,
    AS7341_GAIN_128X,
    AS7341_GAIN_256X,
    AS7341_GAIN_512X,
};

// SMUX configuration commands
enum as7341_smux_cmd
{
    AS7341_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
    AS7341_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
    AS7341_SMUX_CMD_WRITE,     ///< Write SMUX configuration from RAM to SMUX chain
};

// ADC channel specifiers
enum as7341_adc_channel
{
    AS7341_ADC_CHANNEL_0,
    AS7341_ADC_CHANNEL_1,
    AS7341_ADC_CHANNEL_2,
    AS7341_ADC_CHANNEL_3,
    AS7341_ADC_CHANNEL_4,
    AS7341_ADC_CHANNEL_5,
};

// spectral channel specifiers
enum as7341_color_channel_t
{
    AS7341_CHANNEL_F1,
    AS7341_CHANNEL_F2,
    AS7341_CHANNEL_F3,
    AS7341_CHANNEL_F4,
    AS7341_CHANNEL_CLEAR_0,
    AS7341_CHANNEL_NIR_0,
    AS7341_CHANNEL_F5,
    AS7341_CHANNEL_F6,
    AS7341_CHANNEL_F7,
    AS7341_CHANNEL_F8,
    AS7341_CHANNEL_CLEAR,
    AS7341_CHANNEL_NIR,
};

// The number of measurement cycles with spectral data outside of a threshold required to trigger an interrupt
// enum as7341_int_cycle_count
//{
//    AS7341_INT_COUNT_ALL,
//    AS7341_INT_COUNT_1,
//    AS7341_INT_COUNT_2,
//    AS7341_INT_COUNT_3,
//    AS7341_INT_COUNT_5,
//    AS7341_INT_COUNT_10,
//    AS7341_INT_COUNT_15,
//    AS7341_INT_COUNT_20,
//    AS7341_INT_COUNT_25,
//    AS7341_INT_COUNT_30,
//    AS7341_INT_COUNT_35,
//    AS7341_INT_COUNT_40,
//    AS7341_INT_COUNT_45,
//    AS7341_INT_COUNT_50,
//    AS7341_INT_COUNT_55,
//    AS7341_INT_COUNT_60,
//};

// pin directions
enum as7341_gpio_dir
{
    AS7341_GPIO_OUTPUT, ///< THhe GPIO pin is configured as an open drain output
    AS7341_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
};

// wait states for async reading
enum as7341_waiting
{
    AS7341_WAITING_START,
    AS7341_WAITING_LOW,
    AS7341_WAITING_HIGH,
    AS7341_WAITING_DONE,
};

// AGC signal hysteresis
enum as7341_agc_l_level
{
    AGC_L_12,
    AGC_L_25,
    AGC_L_37,
    AGC_L_50
};

enum as7341_agc_h_level
{
    AGC_H_50,
    AGC_H_62,
    AGC_H_75,
    AGC_H_87
};
