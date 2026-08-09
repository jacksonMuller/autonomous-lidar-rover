#pragma once 
#include <cstdint>
#include <string>

class I2C {
public: 
    // Define constructor & deconstructor
    I2C(const std::string& device, uint8_t address); 
    ~I2C(); 

    bool writeRegister(uint8_t reg, uint8_t value); 
private: 
    int fd_;          //num that open() gives
    uint8_t address_; //which device on the bus
};