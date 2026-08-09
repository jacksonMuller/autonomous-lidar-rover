#include "../../include/lidar/lidar.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h> 
#include <unistd.h>

// Constructor
LidarParser::LidarParser(const std::string& port) : port_(port), fd_(-1), valid_(false) {}

// Deconstructor
LidarParser::~LidarParser() {
    if (fd_ != -1) {
        close(fd_); 
    }
}

// Open port method
void LidarParser::openPort() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    // Check if open failed
    if (fd_ < 0) {
        std::cerr << "Opening: " << port_ << " failed!" << std::endl;
        return;
    }

    // Create a termios struct to hold port settings
    struct termios tty;

    // Check if tcgetattr fails 
    if (tcgetattr(fd_, &tty) < 0) {
        std::cerr << "Failed to get port attributes for " << port_ << "." << std::endl;
        close(fd_); 
        fd_ = -1; 
        return;  
    }

    // Set the baud rate to 230400 for both input and output
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

    // Set 8N1: 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;

    // Apply the settings immediately
    tcsetattr(fd_, TCSANOW, &tty);

}

static uint8_t calcCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// Read scan method
void LidarParser::readScan() {
    if (fd_ < 0) {
        valid_ = false;
        scan_.validScan = false;
        return; 
    }

    // Read the bytes until theres a start of a packet (header is 0x54)
    uint8_t byte;

    while (true) {
        ssize_t bytesRead = read(fd_, &byte, 1);

        if (bytesRead != 1) {
            valid_ = false; 
            scan_.validScan = false; 
            return; 
        }

        if (byte == 0x54) {
            break; 
        }
    }

    // Remaining packet is 47 bytes total (including header)
    uint8_t packet[47]; 
    packet[0] = 0x54; 

    ssize_t bytesRead = read(fd_, &packet[1], 46); 

    // Ensure packet size checks out
    if (bytesRead != 46) {
        valid_ = false; 
        scan_.validScan = false;
        return; 
    }

    // Verify the CRC checksum
    uint8_t result = calcCRC8(packet, 46);
    if (result != packet[46]) {
        valid_ = false; 
        scan_.validScan = false; 
        return; 
    }

    // Compute the startAngle (bytes 4 & 5)
    uint16_t rawStart = packet[4] | (packet[5] << 8); 
    float startAngle = rawStart / 100.0f; 

    // Compute the endAngle (bytes 42 and 43)
    uint16_t rawEnd = packet[42] | (packet[43] << 8); 
    float endAngle = rawEnd / 100.0f; 

    // Account for packets that cross 0 degrees
    float angleDiff = endAngle - startAngle;
    
    if (angleDiff < 0.0f) {
        angleDiff += 360.0f; 
    }


    for (int i = 0; i < 12; i++) {
        uint16_t distance = packet[6 + i * 3] | (packet[6 + i * 3 + 1] << 8); 
        uint8_t intensity = packet[6 + i * 3 + 2];

        // Compute the angle
        float angle = startAngle + static_cast<float>(i) * (angleDiff) / 11.0f;

        if (angle >= 360.0f) {
            angle -= 360.0f; 
        }

        int index = static_cast<int>(angle) % 360;
        // Add to scan_.distances (in meters)
        scan_.distances[index] = static_cast<float>(distance) / 1000.0f; 

        // Add intensity to scan_.intensity
        scan_.intensity[index] = intensity; 
    }

    // Record the timestamp (byte 44 & 45)
    scan_.timestamp = packet[44] | (packet[45] << 8);
    scan_.validScan = true;
    valid_ = true;  

}

// validCheck method
bool LidarParser::validCheck() {
    return valid_; 
}

// getLidarData method
LidarScan LidarParser::getLidarData() {
    return scan_; 
}