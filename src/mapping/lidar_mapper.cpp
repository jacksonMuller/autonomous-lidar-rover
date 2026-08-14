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
    if (!std::isfinite(distanceMeters) || distanceMeters < 0.0f || distanceMeters > LidarConstants::MAX_RANGE_M) {
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

    grid_.setCell(gridX, gridY, CellState::Occupied);

    return true;

}