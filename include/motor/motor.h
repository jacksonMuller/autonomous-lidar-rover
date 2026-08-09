#pragma once
#include "../utils/i2c.h"

class MotorController {
public: 
    MotorController(I2C& device);
    void forward(float speed); 
    void backward(float speed); 
    void turnLeft(float speed); 
    void turnRight(float speed); 
    void stop(); 

private: 
    struct MotorChannel {
        int in1; 
        int in2; 
        int pwm; 
    };
    I2C& device_; 
    MotorChannel motors[4]; 

    void setMotor(const MotorChannel& ch, bool in1, bool in2, float speed); 
};