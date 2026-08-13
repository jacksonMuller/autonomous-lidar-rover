#pragma once
#include "types.h"
#include <cstdint>

struct point {
    float x; 
    float y; 

    point(float x, float y): x(x), y(y) {}
}; 

namespace LidarScanUtils {
    point cartesianPosition(float angle, float distance);
    bool validReading(float distance, uint8_t intensity);
    float closestDistance(int startAngle, int endAngle, const LidarScan& scan);
}



