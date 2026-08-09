#pragma once
#include "../utils/types.h"

namespace ScanInjector {
    LidarScan allClear();
    LidarScan wallAhead();
    LidarScan obstacleLeft();
    LidarScan obstacleRight();
    LidarScan surrounded(); 
}