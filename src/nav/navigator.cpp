#include "../../include/nav/navigator.h"
#include "../../include/utils/types.h"
#include "../../include/utils/scan_utils.h"
#include <cmath>

void Navigator::update(const LidarScan& scan) {
    // Determine which action to take
    NavDecision decision = evaluateScan(scan); 

    // Tell the rover which action to take
    switch(decision) {
        case NavDecision::Forward:
            controller_.forward(DRIVE_SPEED);
            break;
        
        case NavDecision::TurnLeft:
            controller_.turnLeft(DRIVE_SPEED); 
            break;
        
        case NavDecision::TurnRight:
            controller_.turnRight(DRIVE_SPEED); 
            break;
        
        case NavDecision::Stop:
        default:
            controller_.stop(); 
            break; 

    }

}

NavDecision Navigator::evaluateScan(const LidarScan& scan) {
    constexpr float DISTANCE_EPSILON = 0.001f; 
    

    if (!scan.validScan) {
        return NavDecision::Stop;
    }

    float forwardClosestDistance = LidarScanUtils::closestDistance(FORWARD_ANGLE_START, FORWARD_ANGLE_END, scan);
    // Check if forwardClosestDistance is infinity
    if (!std::isfinite(forwardClosestDistance)) {
        return NavDecision::Stop;
    }
    
    // Return Forward if the path ahead is clear
    if (forwardClosestDistance >= SAFE_DISTANCE_FORWARD) {
        return NavDecision::Forward; 
    } 

    // Check Left & Right Distances
    float rightClosestDistance = LidarScanUtils::closestDistance(RIGHT_ANGLE_START, RIGHT_ANGLE_END, scan);
    float leftClosestDistance = LidarScanUtils::closestDistance(LEFT_ANGLE_START, LEFT_ANGLE_END, scan); 
    
    
    // Check if rightClosestDistance is infinity
    bool rightValid = std::isfinite(rightClosestDistance);
    // Check if leftClosestDistance is infinity
    bool leftValid = std::isfinite(leftClosestDistance);

    // No trustworthy side readings
    if (!leftValid && !rightValid) {
        return NavDecision::Stop; 
    }

    // Check if right and left side are safe
    bool rightSafe = rightValid && rightClosestDistance > SAFE_DISTANCE_TURN + DISTANCE_EPSILON;
    bool leftSafe = leftValid && leftClosestDistance > SAFE_DISTANCE_TURN + DISTANCE_EPSILON; 

    // Check if neither side is safe
    if (!rightSafe && !leftSafe){
        return NavDecision::Stop;
    }

    // Only right side is safe
    if (rightSafe && !leftSafe) {
        return NavDecision::TurnRight;
    }

    // Only left side is safe
    if (leftSafe && !rightSafe) {
        return NavDecision::TurnLeft;
    }

    // If both are safe, then we chose the side with more clearance
    if (leftClosestDistance > rightClosestDistance) {
        return NavDecision::TurnLeft;
    }

    return NavDecision::TurnRight;
}