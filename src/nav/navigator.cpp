#include "../../include/nav/navigator.h"
#include "../../include/utils/types.h"
#include "../../include/utils/scan_utils.h"
#include <cmath>

void Navigator::update(const LidarScan& scan) {
    // Determine which action to take
    NavigationResult result = evaluateScan(scan); 

    // Sensor failure always puts the rover into a safe state
    if (result.status == NavStatus::SensorFault) {
        state_ = RoverState::SensorFault; 
        controller_.stop();
        return; 
    }

    switch (result.decision) {
        case NavDecision::Forward:
            state_ = RoverState::Navigating;
            controller_.forward(DRIVE_SPEED);
            break;
        
        case NavDecision::TurnLeft:
            state_ = RoverState::AvoidingObstacle;
            controller_.turnLeft(TURN_SPEED); 
            break;
        
        case NavDecision::TurnRight:
            state_ = RoverState::AvoidingObstacle;
            controller_.turnRight(TURN_SPEED); 
            break;

        case NavDecision::Stop:
        default:
            state_ = RoverState::Stopped;
            controller_.stop(); 
            break;
        
    }

}

NavigationResult Navigator::evaluateScan(const LidarScan& scan) {
    constexpr float DISTANCE_EPSILON = 0.001f; 
    

    if (!scan.validScan) {
        return {NavDecision::Stop, NavStatus::SensorFault};
    }

    float forwardClosestDistance = LidarScanUtils::closestDistance(FORWARD_ANGLE_START, FORWARD_ANGLE_END, scan);
    // Check if forwardClosestDistance is infinity
    if (!std::isfinite(forwardClosestDistance)) {
        return {NavDecision::Stop, NavStatus::SensorFault};
    }
    
    // Return Forward if the path ahead is clear
    if (forwardClosestDistance >= SAFE_DISTANCE_FORWARD) {
        return {NavDecision::Forward, NavStatus::Normal}; 
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
        return {NavDecision::Stop, NavStatus::SensorFault}; 
    }

    // Check if right and left side are safe
    bool rightSafe = rightValid && rightClosestDistance > SAFE_DISTANCE_TURN + DISTANCE_EPSILON;
    bool leftSafe = leftValid && leftClosestDistance > SAFE_DISTANCE_TURN + DISTANCE_EPSILON; 

    // If either side has missing sensor data only continue if other side is known to be safe.
    if (!leftValid || !rightValid) {

        // Left is valid and safe then we turn left
        if (leftSafe) {
            return {NavDecision::TurnLeft, NavStatus::Normal}; 
        }

        // Right is valid and safe then we turn right
        if (rightSafe) {
            return {NavDecision::TurnRight, NavStatus::Normal};
        }

        // Missing sensor data prevents us from chosing a trustworthy direction
        return {NavDecision::Stop, NavStatus::SensorFault};
    }

    // Left and Right readings are valid

    // Check if both sides are physically blocked
    if (!leftSafe && !rightSafe) {
        return {NavDecision::Stop, NavStatus::Normal}; 
    }

    // Only right is safe
    if (!leftSafe && rightSafe) {
        return {NavDecision::TurnRight, NavStatus::Normal}; 
    }

    // Only left is safe
    if (leftSafe && !rightSafe) {
        return {NavDecision::TurnLeft, NavStatus::Normal}; 
    }

    // Both sides are safe, so choose greater clearance
    if (leftClosestDistance > rightClosestDistance) {
        return {NavDecision::TurnLeft, NavStatus::Normal}; 
    }

    return {NavDecision::TurnRight, NavStatus::Normal};
}

RoverState Navigator::getState() const {
    return state_; 
}