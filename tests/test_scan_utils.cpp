#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "sim/scan_injector.h"
#include "utils/scan_utils.h"
#include "utils/types.h"

// allClear test (ensure no object is seen) (test #1)
TEST_CASE("allClear scan has no close obstacles") {
    LidarScan scan = ScanInjector::allClear();
    float result = LidarScanUtils::closestDistance(0, 360, scan);
    
    REQUIRE(result > 0.4f);
}

// wallAhead test (ensure obstacle seen in forward arc) (test #2)
TEST_CASE("wallAhead has close obstacle in the forward arc") {
    LidarScan scan = ScanInjector::wallAhead();
    float result = LidarScanUtils::closestDistance(315, 45, scan);

    REQUIRE(result < 0.4f);
}

// obstacleLeft test (ensure no obstacle found in forward arc) (test #3)
TEST_CASE("obstacleLeft (nothing seen in forward arc)") {
    LidarScan scan = ScanInjector::obstacleLeft(); 
    float result = LidarScanUtils::closestDistance(315, 45, scan); 

    REQUIRE (result > 0.4f); 
}

// Test cartesian point correctness (test #4)
TEST_CASE("polarToCartesian test") {
    point result = LidarScanUtils::cartesianPosition(0.0f, 1.0f);

    REQUIRE(result.x == Catch::Approx(1.0f).epsilon(0.1f));
    REQUIRE(result.y == Catch::Approx(0.0f).epsilon(0.1f));
}

// Check for invalid readings (test #5)
TEST_CASE("Ensure invalid readings are properly filtered") {
    LidarScan scan;
    scan.distances[2] = 0.0f;
    scan.distances[4] = 0.0f;

    bool result = true; 

    for (int i = 0; i < 360; i++) {
        bool tempResult = LidarScanUtils::validReading(scan.distances[i], scan.intensity[i]);

        if (tempResult == false) {
            result = false; 
        }
    }

    REQUIRE(result == false); 
}
