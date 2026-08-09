#include "../../include/utils/scan_utils.h"
#include <limits>

point LidarScanUtils::cartesianPosition(float angle, float distance) {
    float angleRadians = angle * (M_PI / 180);

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

    // Convert LiDAR max distance reading (12000mm)
    const float MAX_LIDAR_READ = 12000.0;

    if (distance > MAX_LIDAR_READ || distance <= 0) {
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