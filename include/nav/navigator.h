#pragma once
#include "motor/motor.h"
#include "utils/types.h"


enum class NavDecision {
    Stop,
    Forward,
    TurnLeft,
    TurnRight
};

enum class RoverState {
    Idle,
    Navigating,
    AvoidingObstacle,
    SensorFault,
    Stopped
};

enum class NavStatus {
    Normal,
    SensorFault
}; 

struct NavigationResult {
    NavDecision decision; 
    NavStatus status;
};




class Navigator {

private:
    // Temp value until measurements complete
    static constexpr float SAFE_DISTANCE_FORWARD = 0.4f; 

    // Temp value until measurements complete
    static constexpr float SAFE_DISTANCE_TURN = 0.3f; 

    // Temp value until measurements complete
    static constexpr float SAFE_DISTANCE_REAR = 0.8f; 

    // Temp speed value until measurements complete
    const float DRIVE_SPEED = 0.6f;
    // Temp speed value until measurements complete
    const float TURN_SPEED = 0.5f;


    // Set Forward Angle Arc
    static constexpr int FORWARD_ANGLE_START = 315; 
    static constexpr int FORWARD_ANGLE_END = 45; 

    // Set Right Angle Arc
    static constexpr int RIGHT_ANGLE_START = 225;
    static constexpr int RIGHT_ANGLE_END = 315; 

    // Set Left Angle Arc
    static constexpr int LEFT_ANGLE_START = 45; 
    static constexpr int LEFT_ANGLE_END = 135;

    // Set Rear Angle Arc
    const int REAR_ANGLE_START = 135; 
    const int REAR_ANGLE_END = 225;

    IMotorController& controller_;

    RoverState state_ = RoverState::Idle; 

public: 
    Navigator(IMotorController& controller): controller_(controller) {}

    void update(const LidarScan& scan); 

    static NavigationResult evaluateScan(const LidarScan& scan);

    RoverState getState() const;
};
 