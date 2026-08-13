#include "utils/scan_utils.h"
#include "lidar/lidar_constants.h"

point LidarScanUtils::cartesianPosition(float angleDegrees, float distance) {
    // Convert angle from degrees into radians
    constexpr float PI = 3.14159265358979323846f;
    float angleRadians = (PI / 180) * angleDegrees;

    // Compute x and y values
    float x = distance * std::cos(angleRadians);
    float y = distance * std::sin(angleRadians); 

    return point(x, y);
}

bool LidarScanUtils::validReading(float distance, uint8_t intensity) {
    // Condition returns false on three conditions: 
    // 1) Distance is greater than products max range (12m)
    // 2) Distance is 0
    // 3) intensity is less than 100 (poor scan)

    if (distance > LidarConstants::MAX_RANGE_M || distance <= 0.0f) {
        return false; 
    }

    // Check intensity 
    if (intensity < 100) {
        return false; 
    }

    return true; 
}

float LidarScanUtils::closestDistance(int startAngle, int endAngle, const LidarScan& scan) {
    float closestDistance = std::numeric_limits<float>::infinity();
    int totalSteps = (static_cast<int>(endAngle) - static_cast<int>(startAngle) + 360) % 360;

    for (int i = 0; i < totalSteps; i++) {
        int index = (static_cast<int>(startAngle) + i) % 360; 
        
        // Grab the intensity & distance readings
        uint8_t intensity = scan.intensity[index]; 
        float distance = scan.distances[index]; 

        if (LidarScanUtils::validReading(distance, intensity)) {
            if (distance < closestDistance) {
                closestDistance = distance;
            }
        }
    }

    return closestDistance; 
}