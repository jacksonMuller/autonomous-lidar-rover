#include "../../include/utils/i2c.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifdef __linux__
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#endif


// Define constructor
I2C::I2C(const std::string& device, uint8_t address): fd_(-1), address_(address) {
    fd_ = open(device.c_str(), O_RDWR); 
    
    // Check to see if open failed (fd < 0)
    if (fd_ < 0) {
        // If open fails, return
        return; 
    }
#ifdef __linux__
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        // ioctl failed, close what we opened and reset fd_
        close(fd_); 
        fd_ = -1; 
    }
#endif

};

// Define deconstructor
I2C::~I2C() {
    if (fd_ != -1) {
        close(fd_); 
    }
}; 

// Define writeRegister method
bool I2C::writeRegister(uint8_t reg, uint8_t value) {
    if (fd_ == -1) {
        return false; 
    }

    uint8_t buffer[2] = {reg, value}; 
    int result = write(fd_, buffer, sizeof(buffer)); 

    return result == 2; 

}