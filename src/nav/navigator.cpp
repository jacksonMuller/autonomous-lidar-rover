#include "../../include/nav/navigator.h"
#include "../../include/utils/types.h"
#include "../../include/utils/scan_utils.h"

void Navigator::update(const LidarScan& scan) {
    float forwardClosestDistance = LidarScanUtils::closestDistance(FORWARD_ANGLE_START, FORWARD_ANGLE_END, scan);

    if (forwardClosestDistance <= SAFE_DISTANCE_FORWARD) {
        float leftClosestDistance = LidarScanUtils::closestDistance(LEFT_ANGLE_START, LEFT_ANGLE_END, scan);
        float rightClosestDistance = LidarScanUtils::closestDistance(RIGHT_ANGLE_START, RIGHT_ANGLE_END, scan); 

        if (leftClosestDistance < rightClosestDistance) {
            controller_.turnRight(TURN_SPEED); 
            return; 
        } else {
            controller_.turnLeft(TURN_SPEED);
            return; 
        }
    }

    controller_.forward(DRIVE_SPEED); 

}