#include <catch2/catch_test_macros.hpp>

#include "nav/navigator.h"
#include "sim/scan_injector.h"
#include "utils/types.h"

TEST_CASE("Navigator stops on invalid scan") {
    LidarScan scan{}; 
    scan.validScan = false; 

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::Stop);
}

TEST_CASE("Navigator moves forward when path is clear") {
    LidarScan scan = ScanInjector::allClear();

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::Forward);
}

TEST_CASE("Navigator turns left when left side has more clearance") {
    LidarScan scan{}; 

    // Populate scan distances to unsafe distance (0.3f)
    scan.distances.fill(0.3f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Set left distances to 10.0f (45 to 135 deg) (safe distance)
    for (int i = 45; i <= 135; i++) {
        scan.distances[i] = 10.0f; 
    }

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::TurnLeft);
}

TEST_CASE("Navigator turns right when right side has more clearance") {
    LidarScan scan{}; 

    // Populate scan distances to unsafe distance (0.3f)
    scan.distances.fill(0.3f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Set right distances to 10.0f (225 to 315 deg) (safe distance)
    for (int i = 225; i <= 315; i++) {
        scan.distances[i] = 10.0f;
    }

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::TurnRight);
}

TEST_CASE("Navigator stops when both sides are blocked") {
    LidarScan scan = ScanInjector::surrounded(); 

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::Stop); 
}

TEST_CASE("Navigator stops when forward scan contains no valid readings") {
    LidarScan scan = ScanInjector::allClear();
    scan.validScan = true;

    // Now set forward distances to 0.3f (0 to 45 deg)
    for (int i = 0; i <= 45; i++) {
        scan.distances[i] = 0.0f;
        scan.intensity[i] = 0;  
    }
    // And 315 to 359 deg
    for (int i = 315; i <= 359; i++) {
        scan.distances[i] = 0.0f;
 
    }

    REQUIRE(Navigator::evaluateScan(scan) == NavDecision::Stop);
}


