#pragma once
#include "utils/i2c.h"
#include "i_motor_controller.h"

class MotorController : public IMotorController {
public: 
    MotorController(I2C& device);
    void forward(float speed) override;
    void backward(float speed); 
    void turnLeft(float speed) override; 
    void turnRight(float speed) override; 
    void stop() override; 

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