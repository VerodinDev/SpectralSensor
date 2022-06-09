///********************* EXAMPLE EXTRACTS **************/
//// maybe return a typedef enum
///**
// * @brief Returns the flicker detection status
// *
// * @return int8_t
// */
//int8_t Adafruit_AS7341::getFlickerDetectStatus(void) {
//	Adafruit_BusIO_Register flicker_val =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_FD_STATUS);
//	return (int8_t)flicker_val.read();
//}
//
///**
// * @brief Returns the ADC data for a given channel
// *
// * @param channel The ADC channel to read
// * @return uint16_t The measured data for the currently configured sensor
// */
//uint16_t Adafruit_AS7341::readChannel(as7341_adc_channel_t channel) {
//	// each channel has two uint8_ts, so offset by two for each next channel
//	Adafruit_BusIO_Register channel_data_reg = Adafruit_BusIO_Register(
//		i2c_dev, (AS7341_CH0_DATA_L + 2 * channel), 2, LSBFIRST);
//
//	return channel_data_reg.read();
//}

///**
// * @brief starts the process of getting readings from all channels without using
// * delays
// *
// * @return true: success false: failure (a bit arbitrary)
// */
//bool Adafruit_AS7341::startReading(void) {
//	_readingState = AS7341_WAITING_START; // Start the measurement please
//	checkReadingProgress();               // Call the check function to start it
//	return true;
//}
//
///**
// * @brief runs the process of getting readings from all channels without using
// * delays.  Should be called regularly (ie. in loop()) Need to call
// * startReading() to initialise the process Need to call getAllChannels() to
// * transfer the data into an external buffer
// *
// * @return true: reading is complete false: reading is incomplete (or failed)
// */
//bool Adafruit_AS7341::checkReadingProgress() {
//	if (_readingState == AS7341_WAITING_START) {
//		setSMUXLowChannels(true);        // Configure SMUX to read low channels
//		enableSpectralMeasurement(true); // Start integration
//		_readingState = AS7341_WAITING_LOW;
//		return false;
//	}
//
//	if (!getIsDataReady() || _readingState == AS7341_WAITING_DONE)
//		return false;
//
//	if (_readingState ==
//		AS7341_WAITING_LOW) // Check of getIsDataRead() is already done
//	{
//		Adafruit_BusIO_Register channel_data_reg =
//			Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);
//
//		// bool low_success = channel_data_reg.read((uint8_t *)_channel_readings,
//		// 12);
//		channel_data_reg.read((uint8_t*)_channel_readings, 12);
//
//		setSMUXLowChannels(false);       // Configure SMUX to read high channels
//		enableSpectralMeasurement(true); // Start integration
//		_readingState = AS7341_WAITING_HIGH;
//		return false;
//	}
//
//	if (_readingState ==
//		AS7341_WAITING_HIGH) // Check of getIsDataRead() is already done
//	{
//		_readingState = AS7341_WAITING_DONE;
//		Adafruit_BusIO_Register channel_data_reg =
//			Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);
//		// return low_success &&			//low_success is lost since it
//		// was last call
//		channel_data_reg.read((uint8_t*)&_channel_readings[6], 12);
//		return true;
//	}
//
//	return false;
//}
//
///**
// * @brief transfer all the values from the private result buffer into one
// * nominated
// *
// * @param readings_buffer Pointer to a buffer of length 12 (THERE IS NO ERROR
// * CHECKING, YE BE WARNED!)
// *
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::getAllChannels(uint16_t* readings_buffer) {
//	for (int i = 0; i < 12; i++)
//		readings_buffer[i] = _channel_readings[i];
//	return true;
//}

///**
// * @brief Disable Spectral reading, flicker detection, and power
// *-
// * */
//void Adafruit_AS7341::disableAll(void) {
//	Adafruit_BusIO_Register enable_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
//
//	enable_reg.write(0);
//}

//bool Adafruit_AS7341::enableFlickerDetection(bool enable_fd) {
//
//	Adafruit_BusIO_Register enable_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
//	Adafruit_BusIO_RegisterBits fd_enable_bit =
//		Adafruit_BusIO_RegisterBits(&enable_reg, 1, 6);
//	return fd_enable_bit.write(enable_fd);
//}
//
///**
// * @brief Get the GPIO pin direction setting
// *
// * @return `AS7341_OUTPUT` or `AS7341_INPUT`
// */
//as7341_gpio_dir_t Adafruit_AS7341::getGPIODirection(void) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_input_enable =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 2);
//
//	return (as7341_gpio_dir_t)gpio_input_enable.read();
//}
//
///**
// * @brief Set the GPIO pin to be used as an input or output
// *
// * @param gpio_direction The IO direction to set
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setGPIODirection(as7341_gpio_dir_t gpio_direction) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_input_enable =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 2);
//
//	return gpio_input_enable.write(gpio_direction);
//}
//
///**
// * @brief Get the output inversion setting for the GPIO pin
// *
// * @return true: GPIO output inverted false: GPIO output normal
// */
//bool Adafruit_AS7341::getGPIOInverted(void) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_output_inverted_bit =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 3);
//
//	return gpio_output_inverted_bit.read();
//}
//
///**
// * @brief Invert the logic of then GPIO pin when used as an output
// *
// * @param gpio_inverted **When true** setting the gpio value to **true will
// * connect** the GPIO pin to ground. When set to **false**, setting the GPIO pin
// * value to **true will disconnect** the GPIO pin from ground
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setGPIOInverted(bool gpio_inverted) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_output_inverted_bit =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 3);
//
//	return gpio_output_inverted_bit.write(gpio_inverted);
//}
//
///**
// * @brief Read the digital level of the GPIO pin, high or low
// *
// * @return true: GPIO pin level is high false: GPIO pin level is low
// */
//bool Adafruit_AS7341::getGPIOValue(void) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_input_value_bit =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 0);
//
//	return gpio_input_value_bit.read();
//}
//
///**
// * @brief Set the digital level of the GPIO pin, high or low
// *
// * @param gpio_high The GPIO level to set. Set to true to disconnect the pin
// * from ground. Set to false to connect the gpio pin to ground. This can be used
// * to connect the cathode of an LED to ground to turn it on.
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setGPIOValue(bool gpio_high) {
//	Adafruit_BusIO_Register gpio2_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
//	Adafruit_BusIO_RegisterBits gpio_output_value_bit =
//		Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 1);
//
//	return gpio_output_value_bit.write(gpio_high);
//}

///**
// * @brief Sets the threshold below which spectral measurements will trigger
// * interrupts when the APERS count is reached
// *
// * @param low_threshold the new threshold
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setLowThreshold(uint16_t low_threshold) {
//	Adafruit_BusIO_Register sp_low_threshold_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_SP_LOW_TH_L, 2, LSBFIRST);
//	return sp_low_threshold_reg.write(low_threshold);
//}
//
///**
// * @brief Returns the current low thighreshold for spectral measurements
// *
// * @return int16_t The current low threshold
// */
//uint16_t Adafruit_AS7341::getLowThreshold(void) {
//	Adafruit_BusIO_Register sp_low_threshold_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_SP_LOW_TH_L, 2, LSBFIRST);
//	return sp_low_threshold_reg.read();
//}
//
///**
// * @brief Sets the threshold above which spectral measurements will trigger
// * interrupts when the APERS count is reached
// *
// * @param high_threshold
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setHighThreshold(uint16_t high_threshold) {
//	Adafruit_BusIO_Register sp_high_threshold_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_SP_HIGH_TH_L, 2, LSBFIRST);
//	return sp_high_threshold_reg.write(high_threshold);
//}
//
///**
// * @brief Returns the current high thighreshold for spectral measurements
// *
// * @return int16_t The current high threshold
// */
//uint16_t Adafruit_AS7341::getHighThreshold(void) {
//	Adafruit_BusIO_Register sp_high_threshold_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_SP_HIGH_TH_L, 2, LSBFIRST);
//	return sp_high_threshold_reg.read();
//}
//
///**
// * @brief Enable Interrupts based on spectral measurements
// *
// * @param enable_int true: enable false: disable
// * @return true: success false: falure
// */
//bool Adafruit_AS7341::enableSpectralInterrupt(bool enable_int) {
//	Adafruit_BusIO_Register int_enable_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
//	Adafruit_BusIO_RegisterBits sp_int_bit =
//		Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 3);
//	return sp_int_bit.write(enable_int);
//}
//
///**
// * @brief Enabled system interrupts
// *
// * @param enable_int Set to true to enable system interrupts
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::enableSystemInterrupt(bool enable_int) {
//	Adafruit_BusIO_Register int_enable_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
//	Adafruit_BusIO_RegisterBits sien_int_bit =
//		Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 0);
//	return sien_int_bit.write(enable_int);
//}
//
//// Spectral Interrupt Persistence.
//// Defines a filter for the number of consecutive
//// occurrences that spectral data must remain outside
//// the threshold range between SP_TH_L and
//// SP_TH_H before an interrupt is generated. The
//// spectral data channel used for the persistence filter
//// is set by SP_TH_CHANNEL. Any sample that is
//// inside the threshold range resets the counter to 0.
//
///**
// * @brief Sets the number of times an interrupt threshold must be exceeded
// * before an interrupt is triggered
// *
// * @param cycle_count The number of cycles to trigger an interrupt
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setAPERS(as7341_int_cycle_count_t cycle_count) {
//	Adafruit_BusIO_Register pers_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_PERS);
//	Adafruit_BusIO_RegisterBits apers_bits =
//		Adafruit_BusIO_RegisterBits(&pers_reg, 4, 0);
//	return apers_bits.write(cycle_count);
//}
//
///**
// * @brief Set the ADC channel to use for spectral thresholds including
// * interrupts, automatic gain control, and persistance settings
// *
// * @param channel The channel to use for spectral thresholds. Must be a
// * as7341_adc_channel_t **except for** `AS7341_ADC_CHANNEL_5`
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::setSpectralThresholdChannel(
//	as7341_adc_channel_t channel) {
//	if (channel == AS7341_ADC_CHANNEL_5) {
//		return false;
//	}
//	Adafruit_BusIO_Register cfg_12_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_CFG12);
//	Adafruit_BusIO_RegisterBits spectral_threshold_ch_bits =
//		Adafruit_BusIO_RegisterBits(&cfg_12_reg, 3, 0);
//	return spectral_threshold_ch_bits.write(channel);
//}
//
///**
// * @brief Returns the current value of the Interupt status register
// *
// * @return uint8_t
// */
//uint8_t Adafruit_AS7341::getInterruptStatus(void) {
//	Adafruit_BusIO_Register int_status_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
//	return (uint8_t)int_status_reg.read();
//}
//
///**
// * @brief Returns the status of the spectral measurement threshold interrupts
// *
// * @return true: interrupt triggered false: interrupt not triggered
// */
//bool Adafruit_AS7341::spectralInterruptTriggered(void) {
//	Adafruit_BusIO_Register int_status_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
//	Adafruit_BusIO_RegisterBits aint_bit =
//		Adafruit_BusIO_RegisterBits(&int_status_reg, 1, 3);
//
//	return aint_bit.read();
//}
//
///**
// * @brief Clear the interrupt status register
// *
// * @return true: success false: failure
// */
//bool Adafruit_AS7341::clearInterruptStatus(void) {
//	Adafruit_BusIO_Register int_status_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
//
//	return int_status_reg.write(0xFF);
//}
//
///**
// * @brief The current state of the spectral measurement interrupt status
// * register
// *
// * @return uint8_t The current status register
// */
//uint8_t Adafruit_AS7341::spectralInterruptSource(void) {
//	Adafruit_BusIO_Register status3_reg =
//		Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS3);
//
//	uint8_t spectral_int_source = status3_reg.read();
//	last_spectral_int_source = spectral_int_source;
//	return spectral_int_source;
//}
//
///**
// * @brief The status of the low threshold interrupt
// *
// * @return true: low interrupt triggered false: interrupt not triggered
// */
//bool Adafruit_AS7341::spectralLowTriggered(void) {
//	return (last_spectral_int_source & AS7341_SPECTRAL_INT_LOW_MSK > 0);
//}
//
///**
// * @brief The status of the high threshold interrupt
// *
// * @return true: high interrupt triggered false: interrupt not triggered
// */
//bool Adafruit_AS7341::spectralHighTriggered(void) {
//	return (last_spectral_int_source & AS7341_SPECTRAL_INT_HIGH_MSK > 0);
//}

///**
// * @brief Configure SMUX for flicker detection
// *
// */
//void Adafruit_AS7341::FDConfig() {
//	// SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker
//	// detection
//	writeRegister(uint8_t(0x00), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x01), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x02), uint8_t(0x00)); // reserved/disabled
//	writeRegister(uint8_t(0x03), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x04), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x05), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x06), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x07), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x08), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x09), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x0A), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x0B), uint8_t(0x00)); // Reserved or disabled
//	writeRegister(uint8_t(0x0C), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x0D), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x0E), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x0F), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x10), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x11), uint8_t(0x00)); // disabled
//	writeRegister(uint8_t(0x12), uint8_t(0x00)); // Reserved or disabled
//	writeRegister(uint8_t(0x13), uint8_t(0x60)); // Flicker connected to ADC5 to left of 0x13
//}
//

///**
// * @brief Detect a flickering light
// * @return The frequency of a detected flicker or 1 if a flicker of
// * unknown frequency is detected
// */
//uint16_t Adafruit_AS7341::detectFlickerHz(void) {
//	bool isEnabled = true;
//	bool isFdmeasReady = false;
//
//	// disable everything; Flicker detect, smux, wait, spectral, power
//	disableAll();
//	// re-enable power
//	powerEnable(true);
//
//	// Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
//	// to CFG6)
//	setSMUXCommand(AS7341_SMUX_CMD_WRITE);
//
//	// Write new configuration to all the 20 registers for detecting Flicker
//	FDConfig();
//
//	// Start SMUX command
//	enableSMUX();
//
//	// Enable SP_EN bit
//	enableSpectralMeasurement(true);
//
//	// Enable flicker detection bit
//	writeRegister(uint8_t(AS7341_ENABLE), uint8_t(0x41));
//	delay(500); // SF 2020-08-12 Does this really need to be so long?
//	uint16_t flicker_status = getFlickerDetectStatus();
//	enableFlickerDetection(false);
//	switch (flicker_status) {
//	case 44:
//		return 1;
//	case 45:
//		return 100;
//	case 46:
//		return 120;
//	default:
//		return 0;
//	}
//}
