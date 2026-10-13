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

// Test #34
TEST_CASE("LidarMapper rejects invalid distance measurements") {
    OccupancyGrid grid(250, 250, 0.1f);
    LidarMapper mapper(grid);

    REQUIRE_FALSE(mapper.addMeasurement(0.0f, 0.0f));
    REQUIRE_FALSE(mapper.addMeasurement(0.0f, -1.0f));
    REQUIRE_FALSE(mapper.addMeasurement(0.0f, 12.1f));
}

// Test #35
TEST_CASE("LidarMapper rejects non-finite measurements") {
    OccupancyGrid grid(250, 250, 0.1f);
    LidarMapper mapper(grid);

    REQUIRE_FALSE(mapper.addMeasurement(std::numeric_limits<float>::quiet_NaN(), 1.0f));

    REQUIRE_FALSE(mapper.addMeasurement(0.0f, std::numeric_limits<float>::infinity()));
}

// Test #36
TEST_CASE("Ensure addMeasurement correctly places a horizontal ray traveling to the right") {
    OccupancyGrid grid(250, 250, 0.1f); 
    LidarMapper mapper(grid);

    bool result = mapper.addMeasurement(0.0f, 1.2f);

    REQUIRE(result);

    REQUIRE(grid.getCell(125, 125) == CellState::Free);
    REQUIRE(grid.getCell(130, 125) == CellState::Free);
    REQUIRE(grid.getCell(136, 125) == CellState::Free); 
    // Obstacle endpoint
    REQUIRE(grid.getCell(137, 125) == CellState::Occupied); 
    // Cell behind object
    REQUIRE(grid.getCell(138, 125) == CellState::Unknown);     
}

// Test #37
TEST_CASE("Ensure addMeasurement correctly places a vertical upward ray") {
    OccupancyGrid grid(250, 250, 0.1f);
    LidarMapper mapper(grid); 

    bool result = mapper.addMeasurement(90.0f, 1.0f); 

    REQUIRE(result); 

    REQUIRE(grid.getCell(125, 125) == CellState::Free); 
    REQUIRE(grid.getCell(125, 130) == CellState::Free);
    REQUIRE(grid.getCell(125, 134) == CellState::Free);
    REQUIRE(grid.getCell(125, 135) == CellState::Occupied);
    REQUIRE(grid.getCell(125, 136) == CellState::Unknown);
}

// Test #38
TEST_CASE("Ensure addMeasurement correctly places ray at 45 degrees") {
    OccupancyGrid grid(250, 250, 0.1f);
    LidarMapper mapper(grid); 

    bool result = mapper.addMeasurement(45.0f, 1.0f);
    
    REQUIRE(result);

    REQUIRE(grid.getCell(125, 125) == CellState::Free);
    REQUIRE(grid.getCell(126, 126) == CellState::Free);
    REQUIRE(grid.getCell(127, 127) == CellState::Free); 
    REQUIRE(grid.getCell(128, 128) == CellState::Free); 
    REQUIRE(grid.getCell(129, 129) == CellState::Free); 
    REQUIRE(grid.getCell(130, 130) == CellState::Free); 
    REQUIRE(grid.getCell(131, 131) == CellState::Free); 
    REQUIRE(grid.getCell(132, 132) == CellState::Occupied); 
    REQUIRE(grid.getCell(133, 133) == CellState::Unknown); 

}

// Test #39 
TEST_CASE("Ensure addMeasurement correctly places ray at 225 degrees") {
    OccupancyGrid grid(250, 250, 0.1f); 
    LidarMapper mapper(grid); 

    bool result = mapper.addMeasurement(225.0f, 1.0f); 

    REQUIRE(result); 
    
    REQUIRE(grid.getCell(125, 125) == CellState::Free);
    REQUIRE(grid.getCell(124, 124) == CellState::Free); 
    REQUIRE(grid.getCell(123, 123) == CellState::Free);
    REQUIRE(grid.getCell(122, 122) == CellState::Free); 
    REQUIRE(grid.getCell(121, 121) == CellState::Free); 
    REQUIRE(grid.getCell(120, 120) == CellState::Free); 
    REQUIRE(grid.getCell(119, 119) == CellState::Free); 
    REQUIRE(grid.getCell(118, 118) == CellState::Free); 
    REQUIRE(grid.getCell(117, 117) == CellState::Occupied);
    REQUIRE(grid.getCell(116, 116) == CellState::Unknown); 
}

// Test #40 
TEST_CASE("Ensure addMeasurement correctly places ray at 30 degrees") {
    OccupancyGrid grid(250, 250, 0.1f); 
    LidarMapper mapper(grid); 

    bool result = mapper.addMeasurement(30.0f, 0.9f); 

    REQUIRE(result); 

    REQUIRE(grid.getCell(125, 125) == CellState::Free); 
    REQUIRE(grid.getCell(126, 126) == CellState::Free); 
    REQUIRE(grid.getCell(127, 126) == CellState::Free);
    REQUIRE(grid.getCell(128, 127) == CellState::Free);
    REQUIRE(grid.getCell(129, 127) == CellState::Free); 
    REQUIRE(grid.getCell(130, 128) == CellState::Free); 
    REQUIRE(grid.getCell(131, 128) == CellState::Free); 
    REQUIRE(grid.getCell(132, 129) == CellState::Occupied);


    // Ensure neighboring cell is still unknown
    REQUIRE(grid.getCell(126, 125) == CellState::Unknown); 
    REQUIRE(grid.getCell(133, 130) == CellState::Unknown); 

}