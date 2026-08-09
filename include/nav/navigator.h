#pragma once
#include "motor/motor.h"
#include "utils/types.h"

class Navigator {

private:
    // Temp value until measurements complete
    const float SAFE_DISTANCE_FORWARD = 0.4f; 
    // Temp value until measurements complete
    const float SAFE_DISTANCE_REAR = 0.8f; 

    // Temp speed value until measurements complete
    const float DRIVE_SPEED = 0.6f;
    // Temp speed value until measurements complete
    const float TURN_SPEED = 0.5f;


    // Set Forward Angle Arc
    const int FORWARD_ANGLE_START = 315; 
    const int FORWARD_ANGLE_END = 45; 

    // Set Right Angle Arc
    const int RIGHT_ANGLE_START = 225;
    const int RIGHT_ANGLE_END = 315; 

    // Set Left Angle Arc
    const int LEFT_ANGLE_START = 45; 
    const int LEFT_ANGLE_END = 135;

    // Set Rear Angle Arc
    const int REAR_ANGLE_START = 135; 
    const int REAR_ANGLE_END = 225;

    MotorController& controller_;

public: 
    Navigator(MotorController& controller): controller_(controller) {}

    void update(const LidarScan& scan); 
};
 