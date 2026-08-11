#pragma once
#include <string>
#include "../utils/types.h"

class LidarParser {
private:
    int fd_;
    std::string port_; 
    bool valid_;
    LidarScan scan_;
    bool readExact(uint8_t* buffer, size_t length);

public:
    LidarParser(const std::string& port);
    ~LidarParser(); 
    void openPort(); 
    void readScan(); 
    LidarScan getLidarData(); 
    bool validCheck(); 
}; 