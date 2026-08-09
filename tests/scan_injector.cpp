#include "../include/sim/scan_injector.h"


LidarScan ScanInjector::allClear() {
    LidarScan scan;

    // Populate scan distances to safe distance (10.0f)
    scan.distances.fill(10.0f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true;

    return scan; 
}

LidarScan ScanInjector::wallAhead() {
    LidarScan scan; 
    
    // Populate scan distances to safe distance (10.0f)
    scan.distances.fill(10.0f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Now set forward distances to 0.3f (0 to 45 deg)
    for (int i = 0; i <= 45; i++) {
        scan.distances[i] = 0.3f; 
    }
    // And 315 to 359 deg
    for (int i = 315; i <= 359; i++) {
        scan.distances[i] = 0.3f; 
    }

    return scan; 
}

LidarScan ScanInjector::obstacleLeft() {
    LidarScan scan; 

    // Populate scan distances to safe distance (10.0f)
    scan.distances.fill(10.0f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Set left distances to 0.3f (45 to 135 deg)
    for (int i = 45; i <= 135; i++) {
        scan.distances[i] = 0.3f; 
    }

    return scan;
}

LidarScan ScanInjector::obstacleRight() {
    LidarScan scan; 

    // Populate scan distances to safe distance (10.0f)
    scan.distances.fill(10.0f); 
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150);
    // Set validScan to true
    scan.validScan = true; 

    // Set right distances to 0.3f (225 to 315 deg)
    for (int i = 225; i <= 315; i++) {
        scan.distances[i] = 0.3f;
    }

    return scan;
}

LidarScan ScanInjector::surrounded() {
    LidarScan scan; 

    // Populate scan distances to unsafe distance (0.3f)
    scan.distances.fill(0.3f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150);
    // Set validScan to true 
    scan.validScan = true; 

    return scan;
}