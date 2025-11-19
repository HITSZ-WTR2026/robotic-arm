#include "bsp/iic.h"
// #include <cstdint>
// #include <cstdio>

#ifdef __cplusplus

I2C::I2C(I2C_HandleTypeDef* hi2c)
{
    _hi2c    = hi2c;
    _rxIndex = 0;
}

void I2C::beginTransmission(uint8_t address)
{
    _devAddr = address << 1; // 左移一位以符合HAL库的8位地址格式
    _txBuffer.clear();
}

HAL_StatusTypeDef I2C::endTransmission(bool stop)
{
    // 在这个封装中，我们总是发送停止位，所以忽略stop参数
    // 实际项目中可以根据需要实现
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c, _devAddr, _txBuffer.data(), _txBuffer.size(), HAL_MAX_DELAY);
    return status;
}

uint8_t I2C::write(uint8_t data)
{
    _txBuffer.push_back(data);
    return 1;
}

uint16_t I2C::write(const uint8_t* data, uint16_t quantity)
{
    _txBuffer.insert(_txBuffer.end(), data, data + quantity);
    return quantity;
}

uint8_t I2C::requestFrom(uint8_t address, uint16_t quantity, bool stop)
{
    _devAddr = address << 1; // 左移一位以符合HAL库的8位地址格式
    _rxBuffer.resize(quantity);
    _rxIndex = 0;

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(_hi2c, _devAddr, _rxBuffer.data(), quantity, HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
        return quantity;
    }
    else
    {
        _rxBuffer.clear();
        return 0;
    }
}

int I2C::available()
{
    return _rxBuffer.size() - _rxIndex;
}

uint8_t I2C::read()
{
    if (_rxIndex < _rxBuffer.size())
    {
        return _rxBuffer[_rxIndex++];
    }
    return -1;
}

HAL_StatusTypeDef I2C::memWrite(uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Write(_hi2c, _devAddr, memAddress, memAddSize, pData, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef I2C::memRead(uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size)
{
    return HAL_I2C_Mem_Read(_hi2c, _devAddr, memAddress, memAddSize, pData, size, HAL_MAX_DELAY);
}

#endif // __cplusplus
