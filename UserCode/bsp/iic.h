#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

// #include <stdint.h>
#include <vector>
#include "i2c.h"


#ifdef __cplusplus

/**
 * @brief 一个模仿Arduino Wire库的I2C C++封装类
 */
class I2C
{
public:
    /**
     * @brief 构造函数
     * @param hi2c 指向STM32 HAL I2C句柄的指针
     */
    I2C(I2C_HandleTypeDef* hi2c);

    /**
     * @brief 开始一次I2C传输
     * @param address 7位I2C设备地址
     */
    void beginTransmission(uint8_t address);

    /**
     * @brief 结束I2C传输，将缓冲区中的数据发送出去
     * @param stop (可选) 是否在传输后发送停止位，默认为true
     * @return 0: 成功, 1: 数据过长, 2: 发送地址时收到NACK, 3: 发送数据时收到NACK, 4: 其他错误
     */
    HAL_StatusTypeDef endTransmission(bool stop = true);

    /**
     * @brief 向发送缓冲区写入一个字节
     * @param data 要写入的数据
     * @return 写入的字节数 (总是1)
     */
    uint8_t write(uint8_t data);

    /**
     * @brief 向发送缓冲区写入多个字节
     * @param data 指向数据数组的指针
     * @param quantity 要写入的字节数
     * @return 写入的字节数
     */
    uint16_t write(const uint8_t* data, uint16_t quantity);

    /**
     * @brief 从I2C设备请求数据
     * @param address 7位I2C设备地址
     * @param quantity 要请求的字节数
     * @param stop (可选) 是否在请求后发送停止位，默认为true
     * @return 接收到的字节数
     */
    uint8_t requestFrom(uint8_t address, uint16_t quantity, bool stop = true);

    /**
     * @brief 检查接收缓冲区中是否还有可读数据
     * @return 可读取的字节数
     */
    int available();

    /**
     * @brief 从接收缓冲区读取一个字节
     * @return 读取到的字节；如果缓冲区为空，则返回-1
     */
    uint8_t read();

    HAL_StatusTypeDef memWrite(uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size);
    HAL_StatusTypeDef memRead(uint16_t memAddress, uint16_t memAddSize, uint8_t* pData, uint16_t size);

private:
    I2C_HandleTypeDef* _hi2c;
    uint8_t _devAddr;
    std::vector<uint8_t> _txBuffer;
    std::vector<uint8_t> _rxBuffer;
    size_t _rxIndex;
};

#endif // __cplusplus

#endif // __BSP_IIC_H__
