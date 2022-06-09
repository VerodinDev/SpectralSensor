#pragma once

#include <cstdint>

#define MCP2221_LIB 1
#define DEFAULT_VID 0x04D8
#define DEFAULT_PID 0x00DD

class MCP2221
{
  public:
    MCP2221(uint8_t i2c_address);
    ~MCP2221();

    void connect();
    void printInfo();

    bool writeRegister(uint8_t mem_addr, uint8_t *val, uint16_t size);
    bool writeRegisterByte(uint8_t mem_addr, uint8_t val);
    bool modifyRegisterBit(uint8_t reg, bool value, uint8_t pos);
    uint8_t checkRegisterBit(uint8_t reg, uint8_t pos);
    uint8_t modifyBitInByte(uint8_t var, uint8_t value, uint8_t pos);
    bool modifyRegisterMultipleBit(uint8_t reg, uint8_t value, uint8_t pos, uint8_t bits);
    bool readRegister(uint8_t mem_addr, uint8_t *dest, uint16_t size);
    uint8_t readRegisterByte(uint8_t mem_addr);

    uint8_t m_i2c_address;
    void *m_i2c_dev;

  private:
    MCP2221();
};
