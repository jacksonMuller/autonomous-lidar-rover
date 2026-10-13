#include "mapping/lidar_mapper.h"
#include "mapping/occupancy_grid.h"
#include "lidar/lidar_constants.h"
#include "utils/scan_utils.h"
#include <cmath>


LidarMapper::LidarMapper(OccupancyGrid& grid): grid_(grid) {}

bool LidarMapper::addMeasurement(float angleDegrees, float distanceMeters) {

    // Ensure angle measurement is valid
    if (!std::isfinite(angleDegrees)) {
        return false;
    }

    // Ensure distance reading is valid
    if (!std::isfinite(distanceMeters) || distanceMeters <= 0.0f || distanceMeters > LidarConstants::MAX_RANGE_M) {
        return false; 
    }

    // Convert angle and distance into cartesian points
    point cartPoint = LidarScanUtils::cartesianPosition(angleDegrees, distanceMeters);

    int gridX; 
    int gridY;

    bool converted = grid_.worldToGrid(cartPoint.x, cartPoint.y, gridX, gridY);

    if (!converted) {
        return false;
    }
    
    int roverGridX; 
    int roverGridY; 
    
    bool convertedRoverPos = grid_.worldToGrid(0.0f, 0.0f, roverGridX, roverGridY); 

    if (!convertedRoverPos) {
        return false; 
    }

    traceRay(roverGridX, roverGridY, gridX, gridY); 

    grid_.setCell(gridX, gridY, CellState::Occupied);

    return true;

}

void LidarMapper::traceRay(int startX, int startY, int endX, int endY) {
    int currX = startX; 
    int currY = startY;

    int deltaX = std::abs(endX - startX);
    int deltaY = std::abs(endY - startY); 

    int stepX; 
    int stepY; 

    if (startX < endX) {
        stepX = 1; 
    } else if (startX > endX) {
        stepX = -1; 
    } else {
        stepX = 0; 
    }

    if (startY < endY) {
        stepY = 1; 
    } else if (startY > endY) {
        stepY = -1; 
    } else {
        stepY = 0; 
    }

    int error = deltaX - deltaY; 

    while (currX != endX || currY != endY) {
        if (grid_.isInBounds(currX, currY)) {
            grid_.setCell(currX, currY, CellState::Free);
        }

        int doubledError = 2 * error; 

        if (doubledError > -deltaY) {
            error -= deltaY; 
            currX += stepX;
        }

        if (doubledError < deltaX) {
            error += deltaX;
            currY += stepY;
        }
    }

}