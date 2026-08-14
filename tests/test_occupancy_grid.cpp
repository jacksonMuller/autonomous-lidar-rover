#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "mapping/occupancy_grid.h"

// Test #21
TEST_CASE("OccupancyGrid stores width height and resolution") {
    OccupancyGrid grid(10, 8, 0.1f);

    // Ensure width height and resolution match
    REQUIRE(grid.getWidth() == 10);
    REQUIRE(grid.getHeight() == 8); 
    REQUIRE(grid.getResolution() == Catch::Approx(0.1f)); 
}

// Test #22
TEST_CASE("OccupancyGrid initalizes all cells as Unknown") {
    OccupancyGrid grid(10, 10, 0.1f);


    // Ensure all initalized grid spots are unknown
    for (int y = 0; y < grid.getHeight(); y++) {
        for (int x = 0; x < grid.getWidth(); x++) {
            REQUIRE(grid.getCell(x, y) == CellState::Unknown);
        }
    }
}

// Test #23
TEST_CASE("OccupancyGrid stores cell states and checks bounds") {
    OccupancyGrid grid(10, 10, 0.1f); 

    grid.setCell(4, 6, CellState::Occupied);
    // Ensure the cell has been updated
    REQUIRE(grid.getCell(4, 6) == CellState::Occupied);

    grid.setCell(2, 3, CellState::Free); 
    // Ensure the cell has been updated
    REQUIRE(grid.getCell(2, 3) == CellState::Free); 

    // Ensure isInBounds method functions properly
    REQUIRE(grid.isInBounds(0, 0)); 
    REQUIRE(grid.isInBounds(9, 9));

    REQUIRE_FALSE(grid.isInBounds(-1, 0)); 
    REQUIRE_FALSE(grid.isInBounds(0, -1)); 
    REQUIRE_FALSE(grid.isInBounds(10, 5)); 
    REQUIRE_FALSE(grid.isInBounds(5, 10)); 
}

// Test #24 
TEST_CASE("OccupancyGrid converts world origin to grid center") {
    OccupancyGrid grid(250, 250, 0.1f); 

    int gridX; 
    int gridY; 

    bool result = grid.worldToGrid(0.0f, 0.0f, gridX, gridY); 

    // Ensure result isn't false, as well as gridX and gridY are in the grid center
    REQUIRE(result); 
    REQUIRE(gridX == 125); 
    REQUIRE(gridY == 125);
}

// Test #25 
TEST_CASE("OccupancyGrid converts positive and negative world coordinates") {
    OccupancyGrid grid(250, 250, 0.1f); 

    int gridX; 
    int gridY;

    bool result = grid.worldToGrid(1.2f, -0.7f, gridX, gridY); 

    // Ensure result isn't false as well as gridX and gridY are in the grid center
    REQUIRE(result); 
    REQUIRE(gridX == 137);
    REQUIRE(gridY == 118);
}

// Test #26
TEST_CASE("OccupancyGrid rejects world coordinates outside the grid") {
    OccupancyGrid grid(250, 250, 0.1f); 

    int gridX;
    int gridY;

    REQUIRE_FALSE(grid.worldToGrid(13.0f, 0.0f, gridX, gridY)); 
    REQUIRE_FALSE(grid.worldToGrid(-13.0f, 0.0f, gridX, gridY));
    REQUIRE_FALSE(grid.worldToGrid(0.0f, 13.0f, gridX, gridY));
    REQUIRE_FALSE(grid.worldToGrid(0.0f, -13.0f, gridX, gridY)); 
}

// Test #27
TEST_CASE("OccupancyGrid correctly converts grid coordinates to world coordinates") {
    OccupancyGrid grid(250, 250, 0.1f);

    float worldX;
    float worldY;

    bool result = grid.gridToWorld(125, 125, worldX, worldY);

    // Ensure conversion succeeded
    REQUIRE(result); 
    REQUIRE(worldX == Catch::Approx(0.05f));
    REQUIRE(worldY == Catch::Approx(0.05f)); 
}

// Test #28
TEST_CASE("OccupancyGrid converts another grid cell to world coordinates") {
    OccupancyGrid grid(250, 250, 0.1f);

    float worldX; 
    float worldY; 

    bool result = grid.gridToWorld(137, 118, worldX, worldY);

    // Ensure result isn't false as well as worldX and worldY are in the correct world coordinates
    REQUIRE(result);
    REQUIRE(worldX == Catch::Approx(1.25f));
    REQUIRE(worldY == Catch::Approx(-0.65f));
}

// Test #29
TEST_CASE("OccupancyGrid rejects invalid grid coordinates") {
    OccupancyGrid grid(250, 250, 0.1f); 

    float worldX; 
    float worldY; 

    bool result = grid.gridToWorld(-1, 0, worldX, worldY);
    // Ensure result is false (invalid coordinate)
    REQUIRE_FALSE(result);

    result = grid.gridToWorld(0, -1, worldX, worldY); 
    // Ensure result is false (invalid coordinate)
    REQUIRE_FALSE(result);

    result = grid.gridToWorld(250, 100, worldX, worldY);
    // Ensure result is false (invalid coordinate)
    REQUIRE_FALSE(result); 

    result = grid.gridToWorld(100, 250, worldX, worldY); 
    //Ensure result is false (invalid coordinate)
    REQUIRE_FALSE(result);
}

// Test #30
TEST_CASE("OccupancyGrid round trip conversion test") {
    OccupancyGrid grid(250, 250, 0.1f); 

    int originalX; 
    int originalY;

    bool firstResult = grid.worldToGrid(0.0f, 0.0f, originalX, originalY);
    // Ensure conversion checksout
    REQUIRE(firstResult);
    REQUIRE(originalX == 125);
    REQUIRE(originalY == 125);

    // Convert the grid cell back to world coordinates
    float worldX; 
    float worldY; 

    bool secondResult = grid.gridToWorld(originalX, originalY, worldX, worldY);
    // Ensure conversion checksout and gridToWorld returns the center of the cell
    REQUIRE(secondResult); 
    REQUIRE(worldX == Catch::Approx(0.05f));
    REQUIRE(worldY == Catch::Approx(0.05f)); 

    int finalX; 
    int finalY; 

    bool thridResult = grid.worldToGrid(worldX, worldY, finalX, finalY);
    // Ensure conversion checksout
    REQUIRE(thridResult);
    REQUIRE(finalX == originalX);
    REQUIRE(finalY == originalY);
}
