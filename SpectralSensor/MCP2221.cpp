#include "MCP2221.h"

#include <cstdio>	// printf
#include <iostream>
#include <mcp2221_dll_um.h>

MCP2221::MCP2221()
	: m_i2c_address(0x00)
	, m_i2c_dev(0)
{
}

MCP2221::MCP2221(uint8_t i2c_address)
	: m_i2c_address(i2c_address)
	, m_i2c_dev(0)
{
}

MCP2221::~MCP2221()
{
	//Mcp2221_Close(m_i2c_dev);
}

void MCP2221::connect()
{
	int result(E_ERR_UNKOWN_ERROR);

	//
	unsigned int numberOfDevices(0);
	result = Mcp2221_GetConnectedDevices(DEFAULT_VID, DEFAULT_PID, &numberOfDevices);
	if (result != E_NO_ERR)
	{
		if (result == E_ERR_DEVICE_NOT_FOUND) {
			printf("Mcp2221 device not found\n");
		}
		else {
			printf("Error Mcp2221_GetConnectedDevices %d\n", result);
		}
		
		exit(2);
	}

	std::cout << "Number of devices: " << numberOfDevices << "\n";

	if (numberOfDevices == 0)
	{
		printf("No device connected");
		exit(3);
	}

	//
	unsigned int index(0);
	m_i2c_dev = Mcp2221_OpenByIndex(DEFAULT_VID, DEFAULT_PID, index);
	result = Mcp2221_GetLastError();
	if (result != E_NO_ERR)
	{
		printf("Error Mcp2221_OpenByIndex %d\n", result);
		exit(4);
	}

	//communication speed. Accepted values are between 46875 and 500000
	// 100 kbps if not set              100000
	// AS7341 max 400 KHz				400000
	result = Mcp2221_SetSpeed(m_i2c_dev, 115200);
	if (result != E_NO_ERR)
	{
		exit(5);
	}
}

void MCP2221::printInfo()
{
	int result(E_ERR_UNKOWN_ERROR);

	//
	wchar_t libVersion[64];
	result = Mcp2221_GetLibraryVersion(libVersion);
	if (result != E_NO_ERR)
	{
		exit(1);
	}
	std::wcout << "Library Version: " << libVersion << "\n";

	//
	wchar_t descriptor[31];
	result = Mcp2221_GetManufacturerDescriptor(m_i2c_dev, descriptor);
	if (result != E_NO_ERR)
	{
		exit(4);
	}
	std::wcout << "Manufacturer descriptor: " << descriptor << "\n";

	//
	result = Mcp2221_GetProductDescriptor(m_i2c_dev, descriptor);
	if (result != E_NO_ERR)
	{
		exit(4);
	}
	std::wcout << "Product descriptor: " << descriptor << "\n";

	//
	Mcp2221_GetSerialNumberDescriptor(m_i2c_dev, descriptor);
	if (result != E_NO_ERR)
	{
		exit(4);
	}
	std::wcout << "Serial number: " << descriptor << "\n";

	// 
	result = Mcp2221_GetFactorySerialNumber(m_i2c_dev, descriptor);
	if (result != E_NO_ERR)
	{
		exit(4);
	}
	std::wcout << "Factory serial: " << descriptor << "\n";

}

// note: I2C = big-endian, x86 = little endian
// When writing to these fields, the low byte must be written first, immediately followed by the high byte.
bool MCP2221::writeRegister(uint8_t reg, uint8_t* i2cTxData, uint16_t size)
{
	// append register
	uint8_t data[8 + 1] = { 0x00 };
	data[0] = reg;

	// append data (LSB first)
	// astep 599 = 0x257 -> CA 57 02
	for (uint16_t i = 0; i < size; i++)
	{
		data[i + 1] = i2cTxData[i];
	}

	//uint8_t endi[8 + 1] = { 0x00 };
	//for (uint16_t i = 0; i < size; i++)
	//{
	//	endi[i + 1] = data[size - i];
	//}
	//int result = Mcp2221_I2cWrite(m_i2c_dev, size, m_i2c_address, 1, endi);

	// append data (endian swapped)
	//CA 02 57
	//for (uint16_t i = 0; i < size; i++)
	//{
	//	data[i + 1] = i2cTxData[size - i - 1];
	//}

	int result = Mcp2221_I2cWrite(m_i2c_dev, size, m_i2c_address, 1, data);

	return result == E_NO_ERR;
}

bool MCP2221::writeRegisterByte(uint8_t reg, uint8_t value)
{
	unsigned char i2cTxData[2] = { reg, value };

	int result = Mcp2221_I2cWrite(m_i2c_dev, 2, m_i2c_address, 1, i2cTxData);

	return result == E_NO_ERR;
}

bool MCP2221::modifyRegisterBit(uint8_t reg, bool value, uint8_t pos)
{
	uint8_t register_value = readRegisterByte(reg);
	register_value = modifyBitInByte(register_value, (uint8_t)value, pos);

	return writeRegisterByte(reg, register_value);
}

uint8_t MCP2221::checkRegisterBit(uint8_t reg, uint8_t pos)
{
	uint8_t register_value = readRegisterByte(reg);

	return (uint8_t)((register_value >> pos) & 0x01);
}

bool MCP2221::modifyRegisterMultipleBit(uint8_t reg, uint8_t value, uint8_t pos, uint8_t bits)
{
	uint8_t register_value = readRegisterByte(reg);

	uint8_t mask = (1 << (bits)) - 1;
	value &= mask;

	mask <<= pos;
	register_value &= ~mask;
	register_value |= value << pos;

	return writeRegisterByte(reg, register_value);
}

uint8_t MCP2221::modifyBitInByte(const uint8_t var, const uint8_t value, const uint8_t pos)
{
	uint8_t mask = 1 << pos;
	return ((var & ~mask) | (value << pos));
}

// When reading these fields, the low byte must be read first, and it triggers a 16-bit latch that stores the 16-bit field. The high byte must be read immediately afterwards.
bool MCP2221::readRegister(uint8_t reg, uint8_t* dest, const uint16_t size)
{
	uint8_t i2cRxData[16] = {0};

	int resultw = Mcp2221_I2cWrite(m_i2c_dev, 1, m_i2c_address, 1, &reg);
	int result = Mcp2221_I2cRead(m_i2c_dev, size, m_i2c_address, 1, i2cRxData);

	// prevent endian swapped data
	for (uint16_t i = 0; i < size; i++)
	{
		dest[i] = i2cRxData[i];
	}

	return result == E_NO_ERR;
}

uint8_t MCP2221::readRegisterByte(uint8_t reg)
{
	unsigned char i2cRxData[1];

	int resultw = Mcp2221_I2cWrite(m_i2c_dev, 1, m_i2c_address, 1, &reg);
	int result = Mcp2221_I2cRead(m_i2c_dev, 1, m_i2c_address, 1, i2cRxData);

	if (result != E_NO_ERR)
	{
		printf("readRegisterByte error %d\n", result);
	}

	return i2cRxData[0];
}
