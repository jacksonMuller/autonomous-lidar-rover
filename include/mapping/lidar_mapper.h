#pragma once
#include "occupancy_grid.h"


class LidarMapper {
public: 
    LidarMapper(OccupancyGrid& grid);

    bool addMeasurement(float angleDegrees, float distanceMeters); 


private:
    OccupancyGrid& grid_;
    void traceRay(int startX, int startY, int endX, int endY);

};