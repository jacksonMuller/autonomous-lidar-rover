#include <catch2/catch_test_macros.hpp>

#include "mapping/occupancy_grid.h"
#include "mapping/lidar_mapper.h"

// Test #31
TEST_CASE("LidarMapper marks obstacle endpoint as occupied") {
    OccupancyGrid grid(250, 250, 0.1f);
    LidarMapper mapper(grid);

    bool result = mapper.addMeasurement(0.0f, 1.2f);

    // Ensure mapping succeeded
    REQUIRE(result);

    // Grid cell (137, 125) should be occupied
    REQUIRE(grid.getCell(137, 125) == CellState::Occupied);
    REQUIRE(grid.getCell(138, 125) == CellState::Unknown);

}

// Test #32
TEST_CASE("LidarMapper marks obstacle endpoint as occupied (90 deg)") {
    OccupancyGrid grid(250, 250, 0.1f); 
    LidarMapper mapper(grid); 

    bool result = mapper.addMeasurement(90.0f, 1.0f);

    // Ensure mapping succeeded
    REQUIRE(result); 

    // Grid cell (125, 135) should be occupied
    REQUIRE(grid.getCell(125, 135) == CellState::Occupied); 
    REQUIRE(grid.getCell(125, 136) == CellState::Unknown);
    
}

// Test #33
TEST_CASE("OccupancyGrid converts grid into correct string represntation") {
    OccupancyGrid grid(3, 3, 1.0f);

    // Mark top-right cell as occupied
    grid.setCell(2, 2, CellState::Occupied); 

    // Mark bottom-left cell as free
    grid.setCell(0, 0, CellState::Free);

    std::string output = grid.toString(1, 1, 1);

    std::string expected = "??#\n""???\n"".??\n";

    REQUIRE(output == expected);
}

