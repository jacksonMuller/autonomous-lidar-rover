#pragma once
#include <array>
#include <cstdint>
#include <limits>

struct LidarScan {
    std::array<float, 360> distances;
    std::array<uint8_t, 360> intensity; 
    std::uint64_t timestamp; 
    bool validScan; 


    // Constructor Overload #1
    LidarScan() {
        distances.fill(std::numeric_limits<float>::quiet_NaN());
        intensity.fill(0); 
        timestamp = 0; 
        validScan = false; 

    }


    // Constructor Overload #2
    LidarScan(const std::array<float, 360>& d, const std::array<uint8_t, 360>& i, std::uint64_t ts, bool valid): distances(d), intensity(i), timestamp(ts), validScan(valid) {}

};