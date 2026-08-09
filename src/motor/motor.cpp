#include "../../include/motor/motor.h"
#include <algorithm>

MotorController::MotorController(I2C& device): device_(device) {
    motors[0] = {0, 1, 2}; // front left
    motors[1] = {5, 3, 4}; // rear left
    motors[2] = {6, 7, 8}; // front right
    motors[3] = {11, 9, 10}; // rear right
}

void MotorController::setMotor(const MotorChannel& ch, bool in1, bool in2, float speed) {
    // Clamp speed so that its within 0 - 4095 range
    speed = std::clamp(speed, 0.0f, 1.0f);

    int in1Val = in1 ? 4095 : 0; 
    int in2Val = in2 ? 4095 : 0; 
    int pwmVal = static_cast<int>(speed * 4095); 

    device_.writeRegister(ch.in1, in1Val); 
    device_.writeRegister(ch.in2, in2Val); 
    device_.writeRegister(ch.pwm, pwmVal); 
}

void MotorController::forward(float speed) {
    
    // Configure all motors to go forward 
    setMotor(motors[0], true, false, speed); // front left
    setMotor(motors[1], true, false, speed); // rear left
    setMotor(motors[2], true, false, speed); // front right
    setMotor(motors[3], true, false, speed); // rear right
}

void MotorController::backward(float speed) {

    // Configure all motors to go backwards
    setMotor(motors[0], false, true, speed); // front left
    setMotor(motors[1], false, true, speed); // rear left
    setMotor(motors[2], false, true, speed); // front right
    setMotor(motors[3], false, true, speed); // rear right 
}

void MotorController::turnLeft(float speed) {

    // To turn left, right motors need to go forward and left motors need to go backwards
    setMotor(motors[0], false, true, speed); // front left
    setMotor(motors[1], false, true, speed); // rear left
    setMotor(motors[2], true, false, speed); // front right 
    setMotor(motors[3], true, false, speed); // rear right
}

void MotorController::turnRight(float speed) {

    // To turn right, right motors need to backwards and left motors need to go forward
    setMotor(motors[0], true, false, speed); // front left
    setMotor(motors[1], true, false, speed); // rear left
    setMotor(motors[2], false, true, speed); // front right
    setMotor(motors[3], false, true, speed); // rear right 
}

void MotorController::stop() { 

    // To stop, set all motors to coast with zero power
    setMotor(motors[0], false, false, 0.0f); // front left
    setMotor(motors[1], false, false, 0.0f); // rear left
    setMotor(motors[2], false, false, 0.0f); // front right
    setMotor(motors[3], false, false, 0.0f); // rear right

}