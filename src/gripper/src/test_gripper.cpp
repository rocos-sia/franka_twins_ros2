/**
  ******************************************************************************
  * @file           : test_gripper.cpp
  * @author         : sun
  * @brief          : None
  * @attention      : None
  * @date           : 11/25/24
  ******************************************************************************
  */
#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cstdint>
#include <cstring>

// CRC 计算函数
uint16_t calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < data.size(); i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 打印十六进制数据
void printHex(const std::vector<uint8_t>& data) {
    for (auto byte : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
    }
    std::cout << std::dec << std::endl;
}

// 封装 Modbus RTU 消息
std::vector<uint8_t> createModbusMessage(uint8_t address, uint8_t functionCode, uint16_t registerAddress, uint16_t value) {
    std::vector<uint8_t> message = {
        address,
        functionCode,
        static_cast<uint8_t>(registerAddress >> 8),
        static_cast<uint8_t>(registerAddress & 0xFF),
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    uint16_t crc = calculateCRC(message);
    message.push_back(static_cast<uint8_t>(crc & 0xFF));      // CRC 低字节
    message.push_back(static_cast<uint8_t>(crc >> 8));        // CRC 高字节
    return message;
}

// 示例函数：初始化夹爪
std::vector<uint8_t> initializeGripper(uint8_t address = 0x01) {
    return createModbusMessage(address, 0x06, 0x0100, 0x00A5);
}

// 示例函数：设置力值
std::vector<uint8_t> setForce(uint8_t address, uint16_t forcePercent) {
    return createModbusMessage(address, 0x06, 0x0101, forcePercent);
}

// 示例函数：设置位置
std::vector<uint8_t> setPosition(uint8_t address, uint16_t position) {
    return createModbusMessage(address, 0x06, 0x0103, position);
}

// 示例函数：读取寄存器
std::vector<uint8_t> readRegister(uint8_t address, uint16_t registerAddress, uint16_t numRegisters = 1) {
    std::vector<uint8_t> message = {
        address,
        0x03,
        static_cast<uint8_t>(registerAddress >> 8),
        static_cast<uint8_t>(registerAddress & 0xFF),
        static_cast<uint8_t>(numRegisters >> 8),
        static_cast<uint8_t>(numRegisters & 0xFF)
    };
    uint16_t crc = calculateCRC(message);
    message.push_back(static_cast<uint8_t>(crc & 0xFF));
    message.push_back(static_cast<uint8_t>(crc >> 8));
    return message;
}


int main() {
    // 示例：初始化夹爪
    auto initMessage = initializeGripper();
    std::cout << "Initialization Message: ";
    printHex(initMessage);

    // 示例：设置力值 30% (0x001E)
    auto forceMessage = setForce(0x01, 0x001E);
    std::cout << "Set Force Message: ";
    printHex(forceMessage);

    // 示例：设置位置 500 (0x01F4)
    auto positionMessage = setPosition(0x01, 0x01F4);
    std::cout << "Set Position Message: ";
    printHex(positionMessage);

    // 示例：读取初始化状态
    auto readInitStatusMessage = readRegister(0x01, 0x0200);
    std::cout << "Read Initialization Status Message: ";
    printHex(readInitStatusMessage);

    return 0;
}