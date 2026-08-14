#pragma once
#include "occupancy_grid.h"


class LidarMapper {
public: 
    LidarMapper(OccupancyGrid& grid);

    bool addMeasurement(float angleDegrees, float distanceMeters); 


private:
    OccupancyGrid& grid_;

};